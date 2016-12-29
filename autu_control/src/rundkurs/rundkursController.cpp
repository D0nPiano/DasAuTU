#ifndef _RundkursController_CPP_
#define _RundkursController_CPP_

#include "autu_control/rundkurs/rundkursController.h"
#include "ros/ros.h"

#define STRAIGHT 0
#define CURVE 1

RundkursController::RundkursController(ros::NodeHandle *n,
                                       ros::Publisher *command_pub)
    : n(n), command_pub(command_pub), drivingState(STRAIGHT), pidRegler(*n) {
  ROS_INFO("New RundkursController");

  laser_sub = n->subscribe<sensor_msgs::LaserScan>(
      "/scan", 10, &RundkursController::getCurrentLaserScan, this);

  sensor_sub = n->subscribe<pses_basis::SensorData>(
      "pses_basis/sensor_data", 10, &RundkursController::getCurrentSensorData,
      this);

  laserDetector = std::unique_ptr<LaserDetector>(new LaserDetector());
}

RundkursController::~RundkursController() {
  ROS_INFO("Destroying RundkursController");
  this->stop();
  laser_sub.shutdown();
  sensor_sub.shutdown();
}

void RundkursController::beginCurve() {
  int currentTime = ros::Time::now().toSec();
  if (currentTime - curveBegin > 6) {
    ROS_INFO("Beginning Curve");
    curveBegin = ros::Time::now().toSec();
  } else {
    ROS_INFO("Curve was not too long ago...");
  }
}

void RundkursController::driveCurve() {
  command_data cmd;

  static float driveStraightTime;
  static float driveCurveTime;
  static float cornerBeginAngle;

  float curveTimer = ros::Time::now().toSec() - curveBegin;

  if (curveTimer < 0.3) {
    float ldist = currentSensorData->range_sensor_left;
    driveStraightTime = 0.0 + (0.5 * ldist);
    cornerBeginAngle = laserDetector->getAngleToWall();
    // driveStraightTime = 0.0;

    float curveSeconds = 0.4 + (cornerBeginAngle / PI / 2) * 6.0;
    driveCurveTime = driveStraightTime + .6 + curveSeconds;

    ROS_INFO("CurveCompleted: cornerBeginAngle: [%f]",
             (cornerBeginAngle * 180 / PI));
    ROS_INFO("CurveCompleted: driveStraightTime: [%f]", driveStraightTime);
    ROS_INFO("CurveCompleted: driveCurveTime: [%f]", driveCurveTime);
  }

  if (curveTimer < driveStraightTime) {
    cmd.motor_level = 10;
    cmd.steering_level = 0;
  } else if (curveTimer > driveCurveTime - 1.1 && curveTimer < driveCurveTime &&
             laserDetector->getAngleToWallInDeg() < 95.0) {
    ROS_INFO("Kurvenfahrt beendet");
    cmd.motor_level = 10;
    cmd.steering_level = 0;
  } else if (curveTimer < driveCurveTime) {
    cmd.motor_level = 7;
    cmd.steering_level = 30;
  } else {
    cmd.motor_level = 10;
    cmd.steering_level = 0;
  }

  cmd.header.stamp = ros::Time::now();
  command_pub->publish(cmd);
  ros::spinOnce();
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

void RundkursController::simpleController() {
  switch (drivingState) {
  case STRAIGHT:
    pidRegler.drive(currentSensorData->range_sensor_left,
                    laserDetector->getDistanceToWall());
    if (laserDetector->isNextToCorner()) {
      ROS_INFO("************ Corner ***************");
      beginCurve();
      ROS_INFO("Seconds: [%f]", curveBegin);
      drivingState = CURVE;
    }
    break;
  case CURVE:
    this->driveCurve();
    if (laserDetector->isNextToWall() &&
        (curveBegin + RundkursController_MAX_CURVE_SECONDS) <
            ros::Time::now().toSec() &&
        (laserDetector->getAngleToWall() * 180 / PI) < 100.0)
      drivingState = STRAIGHT;
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
        currentLaserScan->ranges[0] == -1.0 ||
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
