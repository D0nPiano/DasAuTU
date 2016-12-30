#ifndef _RundkursController_CPP_
#define _RundkursController_CPP_

#include "autu_control/rundkurs/rundkursController.h"
#include "ros/ros.h"

#define STRAIGHT 0
#define CURVE 1

RundkursController::RundkursController(ros::NodeHandle *n,
                                       ros::Publisher *command_pub)
    : n(n), command_pub(command_pub), drivingState(STRAIGHT), pidRegler(*n),
      curveDriver(*n) {
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
}

void RundkursController::odomCallback(const nav_msgs::OdometryConstPtr &msg) {
  odomData = msg;
}

void RundkursController::simpleController() {
  switch (drivingState) {
  case STRAIGHT:
    // if (laserDetector->isNextToCorner()) {
    ROS_INFO("************ Corner ***************");
    curveDriver.reset();
    curveDriver.curveInit(0.8, true, odomData);
    drivingState = CURVE;
    // }
    break;
  case CURVE:
    if (curveDriver.isAroundTheCorner() && pidRegler.isReady())
      drivingState = STRAIGHT;
    break;
  default:
    break;
  }

  switch (drivingState) {
  case STRAIGHT:
    pidRegler.drive(currentSensorData->range_sensor_left,
                    laserDetector->getDistanceToWall());
    break;
  case CURVE:
    // curveDriver.drive(currentSensorData->range_sensor_left,
    //                  laserDetector->getAngleToWall());
    curveDriver.drive(odomData);
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
