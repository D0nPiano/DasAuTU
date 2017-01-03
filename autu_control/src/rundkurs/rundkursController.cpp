#ifndef _RundkursController_CPP_
#define _RundkursController_CPP_

#include "autu_control/rundkurs/rundkursController.h"
#include "ros/ros.h"

#ifndef NDEBUG
#include "std_msgs/Float32.h"
#endif

#define STRAIGHT 0
#define CURVE 1
#define BEFORE_CURVE 2

RundkursController::RundkursController(ros::NodeHandle *n,
                                       ros::Publisher *command_pub)
    : n(n), command_pub(command_pub), laserUtil(*n), lowpass(10),
      drivingState(STRAIGHT), pidRegler(*n), curveDriver(*n) {
  ROS_INFO("New RundkursController");

  laser_sub = n->subscribe<sensor_msgs::LaserScan>(
      "/scan", 10, &RundkursController::getCurrentLaserScan, this);

  sensor_sub = n->subscribe<pses_basis::SensorData>(
      "pses_basis/sensor_data", 10, &RundkursController::getCurrentSensorData,
      this);

  odom_sub = n->subscribe<nav_msgs::Odometry>(
      "/odom", 1, &RundkursController::odomCallback, this);

  laserDetector = std::unique_ptr<LaserDetector>(new LaserDetector());
  pidRegler.setLaserDetector(*laserDetector);

#ifndef NDEBUG
  us_raw_dbg_pub = n->advertise<std_msgs::Float32>("autu/debug/us_raw", 1);
  us_lp_dbg_pub = n->advertise<std_msgs::Float32>("autu/debug/us_lp", 1);
  ransac_dbg_pub = n->advertise<std_msgs::Float32>("autu/debug/ransac", 1);
#endif
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
  laserDetector->setCurrentLaserScan(msg);
}

void RundkursController::getCurrentSensorData(
    const pses_basis::SensorData::ConstPtr &msg) {
  currentSensorData = msg;
  lowpass.addValue(currentSensorData->range_sensor_left);
}

void RundkursController::odomCallback(const nav_msgs::OdometryConstPtr &msg) {
  odomData = msg;
}

float RundkursController::getDistanceToWall() {
  const float laserDist = laserUtil.getDistanceToWall(currentLaserScan, true);
  if (laserDist == -1 || laserDist > 5)
    return currentSensorData->range_sensor_left;
  else
    return (laserDist + currentSensorData->range_sensor_left) / 2;
}

void RundkursController::simpleController() {
  curveDriver.setLaserscan(currentLaserScan);
  curveDriver.setOdom(odomData);
// ROS_INFO("Distance to Wall: %f",
//          laserUtil.getDistanceToWall(currentLaserScan, true));

#ifndef NDEBUG
  std_msgs::Float32 value;

  value.data = currentSensorData->range_sensor_left;
  us_raw_dbg_pub.publish(value);

  value.data = lowpass.getAverage();
  us_lp_dbg_pub.publish(value);

  value.data = laserUtil.getDistanceToWall(currentLaserScan, true);
  ransac_dbg_pub.publish(value);
#endif

  switch (drivingState) {
  case STRAIGHT:
    if (curveDriver.isNextToCorner(true)) {
      ROS_INFO("************ Next To Corner ***************");
      curveDriver.reset();
      drivingState = BEFORE_CURVE;
    }
    break;
  case BEFORE_CURVE:
    if (curveDriver.isAtCurveBegin(true)) {
      ROS_INFO("************ Corner ***************");
      curveDriver.curveInit(1.8, true);
      drivingState = CURVE;
    }
    break;
  case CURVE:
    if (curveDriver.isAroundTheCorner()) { //&& pidRegler.isReady()){
      pidRegler.reset();
      drivingState = STRAIGHT;
    }
    break;
  default:
    break;
  }

  switch (drivingState) {
  case STRAIGHT:
    // pidRegler.drive(currentSensorData->range_sensor_left,
    //              laserDetector->getDistanceToWall());
    pidRegler.drive(lowpass.getAverage());
    break;
  case BEFORE_CURVE:
    /* pidRegler.drive((currentSensorData->range_sensor_left +
                      laserDetector->getDistanceToWall()) /
                     2);*/
    pidRegler.drive(lowpass.getAverage());
    break;
  case CURVE:
    // curveDriver.drive(currentSensorData->range_sensor_left,
    //                  laserDetector->getAngleToWall());
    curveDriver.drive();
    break;
  default:
    break;
  }
  // ROS_INFO("Angle to wall in deg: [%f]", laserDetector->getAngleToWall() *
  // 180 / PI);
}

void RundkursController::run() {
  if (!initialized) {
    if (currentLaserScan == nullptr || currentSensorData == nullptr ||
        odomData == nullptr || currentLaserScan->ranges[0] == -1.0 ||
        currentSensorData->range_sensor_left == -1.0) {
      ROS_INFO("Sensors Uninitialized!");
      return;
    } else {
      initialized = true;
      laserDetector->initialize();
    }
  }

  // If everything is initialized, run Controller
  this->simpleController();
}

#endif
