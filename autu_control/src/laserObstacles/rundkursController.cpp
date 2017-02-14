#ifndef _LaserObstaclesController_CPP_
#define _LaserObstaclesController_CPP_

#include "autu_control/laserObstacles/rundkursController.h"
#include "ros/ros.h"

#ifndef NDEBUG
#include "std_msgs/Float32.h"
#endif

#define STRAIGHT 0
#define CURVE 1
#define STUPIDSTRAIGHT 2
#define STRAIGHTOBSTACLE 3

LaserObstaclesController::LaserObstaclesController(ros::NodeHandle *n,
                                       ros::Publisher *command_pub)
    : n(n), command_pub(command_pub), laserUtil(*n), lowpass(10),
      drivingState(STRAIGHT), pidRegler(*n), curveDriver(*n, laserUtil),
      time_of_last_corner(0) {
  ROS_INFO("New LaserObstaclesController");

  laser_sub = n->subscribe<sensor_msgs::LaserScan>(
      "/scan", 10, &LaserObstaclesController::getCurrentLaserScan, this);

  sensor_sub = n->subscribe<pses_basis::SensorData>(
      "pses_basis/sensor_data", 10, &LaserObstaclesController::getCurrentSensorData,
      this);

  odom_sub = n->subscribe<nav_msgs::Odometry>(
      "/odom", 1, &LaserObstaclesController::odomCallback, this);

  carinfo_sub = n->subscribe<pses_basis::CarInfo>(
      "/pses_basis/car_info", 1, &LaserObstaclesController::carinfoCallback, this);

  laserDetector = std::unique_ptr<LaserDetector>(new LaserDetector());
  pidRegler.setLaserDetector(*laserDetector);

  pd_maxMotorLevel = pidRegler.getMaxMotorLevel();

  curveRadius = n->param<float>("main/curvedriver/curve_radius", 1.8f);

#ifndef NDEBUG
  us_raw_dbg_pub = n->advertise<std_msgs::Float32>("autu/debug/us_raw", 1);
  us_lp_dbg_pub = n->advertise<std_msgs::Float32>("autu/debug/us_lp", 1);
  ransac_dbg_pub = n->advertise<std_msgs::Float32>("autu/debug/ransac", 1);
#endif
}

LaserObstaclesController::~LaserObstaclesController() {
  ROS_INFO("Destroying LaserObstaclesController");
  this->stop();
  laser_sub.shutdown();
  sensor_sub.shutdown();
}

void LaserObstaclesController::stop() {
  command_data cmd;
  cmd.motor_level = 0;
  cmd.steering_level = 0;
  cmd.header.stamp = ros::Time::now();
  command_pub->publish(cmd);
  ros::spinOnce();
}

void LaserObstaclesController::getCurrentLaserScan(
    const sensor_msgs::LaserScan::ConstPtr &msg) {
  currentLaserScan = msg;
  laserDetector->setCurrentLaserScan(msg);
}

void LaserObstaclesController::getCurrentSensorData(
    const pses_basis::SensorData::ConstPtr &msg) {
  currentSensorData = msg;
  lowpass.addValue(currentSensorData->range_sensor_left);
}

void LaserObstaclesController::odomCallback(const nav_msgs::OdometryConstPtr &msg) {
  odomData = msg;
}

void LaserObstaclesController::carinfoCallback(
    const pses_basis::CarInfoConstPtr &msg) {
  currentCarInfo = msg;
}

float LaserObstaclesController::getDistanceToWall() {
  const float laserDist = laserUtil.getDistanceToWall(currentLaserScan, true);
  if (laserDist == -1 || laserDist > 5)
    return currentSensorData->range_sensor_left;
  else
    return (laserDist + currentSensorData->range_sensor_left) / 2;
}

void LaserObstaclesController::simpleController() {
  curveDriver.setLaserscan(currentLaserScan);
  curveDriver.setOdom(odomData);
/*
  static ros::Time time = ros::Time::now();
  if (ros::Time::now().toSec() - time.toSec() > 0.1) {
    // ROS_INFO("NextToCorner: %d",
    curveDriver.isNextToCorner(true, currentCarInfo->speed); //);
    time = ros::Time::now();
  }
  return;*/
#ifndef NDEBUG
  std_msgs::Float32 value;

  value.data = currentSensorData->range_sensor_left;
  us_raw_dbg_pub.publish(value);

  value.data = lowpass.getAverage();
  us_lp_dbg_pub.publish(value);

// value.data = laserUtil.getDistanceToWall(currentLaserScan, true);
// ransac_dbg_pub.publish(value);
#endif
  

  switch (drivingState) {
  case STRAIGHT:
  case STRAIGHTOBSTACLE:
    if (ros::Time::now().toSec() - time_of_last_corner > 1.5) {
      if (currentSensorData->range_sensor_left > 1.6) {
        ROS_INFO("************ Corner ***************");
        curveDriver.reset();
        curveDriver.curveInit(curveRadius, true);
        drivingState = CURVE;
      }
    }
    break;
  case CURVE:
    if (curveDriver.isAroundTheCorner(currentLaserScan)) {
      //&& pidRegler.isReady(lowpass.getAverage())) {
      ROS_INFO("************ End of Corner ***************");
      time_of_last_corner = ros::Time::now().toSec();

      pses_basis::Command cmd;
      cmd.motor_level = 10;
      cmd.steering_level = 25;
      cmd.header.stamp = ros::Time::now();
      command_pub->publish(cmd);
      ros::spinOnce();

      drivingState = STUPIDSTRAIGHT;
    }
    break;

  case STUPIDSTRAIGHT:
    if (ros::Time::now().toSec() - time_of_last_corner > 0.4) {
      pses_basis::Command cmd;
      cmd.motor_level = 10;
      cmd.steering_level = -25;
      cmd.header.stamp = ros::Time::now();
      command_pub->publish(cmd);
      ros::spinOnce();
    }
    if (ros::Time::now().toSec() - time_of_last_corner > 0.8) {
      //&& pidRegler.isReady(lowpass.getAverage())) {
      ROS_INFO("************ End of Straight ***************");
      pidRegler.reset();
      drivingState = STRAIGHT;
    }
    break;
  default:
    break;
  }
  

  float frontDistance = getFrontObstacleDist();
  switch (drivingState) {
  case STRAIGHT:
    if(frontDistance != -1){
      bool isRight = laserDetector->isObstacleRight();
      pidRegler.avoidDirection = !isRight;
      if(isRight){
        ROS_INFO("************ Obstacle Right ***************");
      } else {
        ROS_INFO("************ Obstacle Left ***************");        
      }
      pidRegler.avoidObstacle(lowpass.getAverage());
      drivingState = STRAIGHTOBSTACLE;
    }
    pidRegler.setMaxMotorLevel(pd_maxMotorLevel);
    pidRegler.drive(lowpass.getAverage(), true);
    break;
  case STRAIGHTOBSTACLE:
    pidRegler.setMaxMotorLevel(5);
    pidRegler.drive(lowpass.getAverage(), true);
    if(frontDistance != -1 && frontDistance < 3.0){
      ROS_INFO("Obstacle");
      pidRegler.avoidObstacle(lowpass.getAverage());
      pidRegler.setMaxMotorLevel(7 + (int)(frontDistance));
      pidRegler.drive(lowpass.getAverage(), true);
    } else {
      drivingState = STRAIGHT;
    }
    break;
  case CURVE:
    curveDriver.drive();
    break;
  default:
    break;
  }
}

float LaserObstaclesController::getFrontObstacleDist(){
  float laserDist = laserDetector->getFrontObstacleDist();
  if(laserDist > 2.0){
    laserDist = -1.0;
  }
  return laserDist;
}

void LaserObstaclesController::run() {
  if (!initialized) {
    if (currentLaserScan == nullptr || currentSensorData == nullptr ||
        odomData == nullptr || currentCarInfo == nullptr ||
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
  this->getFrontObstacleDist();
  this->simpleController();
}

#endif
