#ifndef _RundkursController_CPP_
#define _RundkursController_CPP_

#include "autu_control/rundkurs/rundkursController.h"
#include "ros/ros.h"

#define STRAIGHT 0
#define BEFORE_CURVE 1
#define CURVE 2
#define ROLLOUT 3

RundkursController::RundkursController(ros::NodeHandle *n,
                                       ros::Publisher *command_pub)
    : n(n), commandPub(command_pub), laserUtil(*n), curveDriver(*n, laserUtil),
      lowpass(10), pdController(*n), drivingState(STRAIGHT),
      timeOfLastCorner(0) {
  ROS_INFO("New CircuitController");

  laserscanSub = n->subscribe<sensor_msgs::LaserScan>(
      "/scan", 10, &RundkursController::getCurrentLaserScan, this);

  sensorDataSub = n->subscribe<pses_basis::SensorData>(
      "pses_basis/sensor_data", 10, &RundkursController::getCurrentSensorData,
      this);

  odomSub = n->subscribe<nav_msgs::Odometry>(
      "/odom", 1, &RundkursController::odomCallback, this);

  carinfoSub = n->subscribe<pses_basis::CarInfo>(
      "/pses_basis/car_info", 1, &RundkursController::carinfoCallback, this);

  pdMaxMotorLevel = pdController.getMaxMotorLevel();

  afterCurveDeadtime =
      n->param<double>("main/curvedriver/after_curve_deadtime", 1.5);
}

RundkursController::~RundkursController() {
  ROS_INFO("Destroying RundkursController");
  this->stop();
  laserscanSub.shutdown();
  sensorDataSub.shutdown();
  carinfoSub.shutdown();
  odomSub.shutdown();
}

void RundkursController::stop() {
  pses_basis::Command cmd;
  cmd.motor_level = 0;
  cmd.steering_level = 0;
  cmd.header.stamp = ros::Time::now();
  commandPub->publish(cmd);
  ros::spinOnce();
}

void RundkursController::getCurrentLaserScan(
    const sensor_msgs::LaserScan::ConstPtr &msg) {
  currentLaserScan = msg;
}

void RundkursController::getCurrentSensorData(
    const pses_basis::SensorData::ConstPtr &msg) {
  currentSensorData = msg;
  // update the ultrasonic lowpass
  lowpass.addValue(currentSensorData->range_sensor_left);
}

void RundkursController::odomCallback(const nav_msgs::OdometryConstPtr &msg) {
  odomData = msg;
}

void RundkursController::carinfoCallback(
    const pses_basis::CarInfoConstPtr &msg) {
  currentCarInfo = msg;
}

void RundkursController::controlCar() {
  // update the curveDriver with the current data
  curveDriver.setLaserscan(currentLaserScan);
  curveDriver.setOdom(odomData);

  // FSM transitions
  switch (drivingState) {
  case STRAIGHT:

    // The transition from STRAIGHT to BEFORE_CURVE is only possible, if the
    // last curve wasn't shortly before
    if (ros::Time::now().toSec() - timeOfLastCorner > afterCurveDeadtime)
      // curveDriver checks if the car is near to a corner
      if (curveDriver.isNextToCorner(lowpass.getAverage(),
                                     currentCarInfo->speed)) {
        ROS_INFO("************ Next To Corner ***************");
        drivingState = BEFORE_CURVE;
      }

    break;
  case BEFORE_CURVE:

    if (curveDriver.rolloutBegins()) {
      ROS_INFO("************ Rollout ***************");
      drivingState = ROLLOUT;
    }

    break;
  case ROLLOUT:

    if (curveDriver.isAtCurveBegin()) {
      ROS_INFO("************ Corner ***************");
      curveDriver.curveInit();
      drivingState = CURVE;
    }

    break;
  case CURVE:

    if (curveDriver.isAroundTheCorner()) {
      ROS_INFO("************ End of Corner ***************");
      timeOfLastCorner = ros::Time::now().toSec();
      pdController.reset();
      drivingState = STRAIGHT;
    }

    break;
  default:
    break;
  }

  // FSM output
  switch (drivingState) {
  case STRAIGHT:

    // let the pd-controller drive with its maximum speed
    pdController.setMaxMotorLevel(pdMaxMotorLevel);
    pdController.drive(lowpass.getAverage(), true);

    break;
  case BEFORE_CURVE:

    // let the pd-controller drive with its maximum speed
    pdController.setMaxMotorLevel(pdMaxMotorLevel);
    pdController.drive(lowpass.getAverage(), true);

    break;
  case ROLLOUT:

    // turn off the motor but let the pd-controller control the steering
    // motorlevel must not be zero because this would freak out the odometry
    pdController.setMaxMotorLevel(1);
    pdController.drive(lowpass.getAverage(), true);

    break;
  case CURVE:

    // let the curvedriver drive in the curve
    curveDriver.drive();

    break;
  default:

    // this case should never occur
    // stop the car immediatly because the FSM is broken
    stop();

    break;
  }
}

void RundkursController::run() {
  // check if all required sensors are ready
  if (!initialized) {
    /*
            If LaserScan is uninitialized, its range[0] is -1.0
            If SensorData is uninitialized, its range_sensor_left is -1.0
    */
    if (currentLaserScan == nullptr || currentSensorData == nullptr ||
        odomData == nullptr || currentCarInfo == nullptr ||
        currentLaserScan->ranges[0] == -1.0 ||
        currentSensorData->range_sensor_left == -1.0) {
      ROS_INFO("Sensors Uninitialized!");
      return;
    } else {
      initialized = true;
    }
  }

  // If everything is initialized, run Controller
  this->controlCar();
}

#endif
