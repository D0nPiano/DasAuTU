#ifndef _RundkursController_CPP_
#define _RundkursController_CPP_

#include "autu_control/rundkurs/rundkursController.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"

#define STRAIGHT 0
#define BEFORE_CURVE 1
#define CURVE 2
#define ROLLOUT 3

RundkursController::RundkursController(ros::NodeHandle *n,
                                       ros::Publisher *command_pub)
    : n(n), command_pub(command_pub), laserUtil(*n), lowpass(10),
      drivingState(STRAIGHT), pidRegler(*n), curveDriver(*n, laserUtil),
      time_of_last_corner(0) {
  ROS_INFO("New RundkursController");

  laser_sub = n->subscribe<sensor_msgs::LaserScan>(
      "/scan", 10, &RundkursController::getCurrentLaserScan, this);

  sensor_sub = n->subscribe<pses_basis::SensorData>(
      "pses_basis/sensor_data", 10, &RundkursController::getCurrentSensorData,
      this);

  odom_sub = n->subscribe<nav_msgs::Odometry>(
      "/odom", 1, &RundkursController::odomCallback, this);

  carinfo_sub = n->subscribe<pses_basis::CarInfo>(
      "/pses_basis/car_info", 1, &RundkursController::carinfoCallback, this);

  pd_maxMotorLevel = pidRegler.getMaxMotorLevel();

  curveRadius = n->param<float>("main/curvedriver/curve_radius", 1.8f);

  after_curve_deadtime =
      n->param<double>("main/curvedriver/after_curve_deadtime", 1.5);
}

RundkursController::~RundkursController() {
  ROS_INFO("Destroying RundkursController");
  this->stop();
  laser_sub.shutdown();
  sensor_sub.shutdown();
}

void RundkursController::stop() {
  command_data cmd;
  cmd.motor_level = 0;
  cmd.steering_level = 0;
  cmd.header.stamp = ros::Time::now();
  command_pub->publish(cmd);
  ros::spinOnce();
}

void RundkursController::getCurrentLaserScan(
    const sensor_msgs::LaserScan::ConstPtr &msg) {
  currentLaserScan = msg;
}

void RundkursController::getCurrentSensorData(
    const pses_basis::SensorData::ConstPtr &msg) {
  currentSensorData = msg;
  lowpass.addValue(currentSensorData->range_sensor_left);
}

void RundkursController::odomCallback(const nav_msgs::OdometryConstPtr &msg) {
  odomData = msg;
}

void RundkursController::carinfoCallback(
    const pses_basis::CarInfoConstPtr &msg) {
  currentCarInfo = msg;
}

void RundkursController::simpleController() {
  curveDriver.setLaserscan(currentLaserScan);
  curveDriver.setOdom(odomData);

  switch (drivingState) {
  case STRAIGHT:
    if (ros::Time::now().toSec() - time_of_last_corner > after_curve_deadtime) {
      if (curveDriver.isNextToCorner(currentCarInfo->speed)) {
        ROS_INFO("************ Next To Corner ***************");
        drivingState = BEFORE_CURVE;
      }
    }
    break;
  case BEFORE_CURVE:
    if (curveDriver.rolloutBegins()) {
      ROS_INFO("************ Rollout ***************");
      curveDriver.reset();
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
      time_of_last_corner = ros::Time::now().toSec();
      pidRegler.reset();
      drivingState = STRAIGHT;
    }
    break;
  default:
    break;
  }

  switch (drivingState) {
  case STRAIGHT:
    pidRegler.setMaxMotorLevel(pd_maxMotorLevel);
    pidRegler.drive(lowpass.getAverage(), true);
    break;
  case BEFORE_CURVE:
    pidRegler.setMaxMotorLevel(pd_maxMotorLevel);
    pidRegler.drive(lowpass.getAverage(), true);
    break;
  case ROLLOUT:
    pidRegler.setMaxMotorLevel(1);
    pidRegler.drive(lowpass.getAverage(), true);
    break;
  case CURVE:
    curveDriver.drive();
    break;
  default:
    stop();
    break;
  }
}

void RundkursController::run() {
  if (!initialized) {
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
  this->simpleController();
}

#endif
