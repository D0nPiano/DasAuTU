#ifndef _SimpleObstacleController_CPP_
#define _SimpleObstacleController_CPP_

#include "autu_control/simpleObstacle/simpleObstacleController.h"
#include "ros/ros.h"
#include <math.h>       /* pow */

#define CAR_WIDTH 0.2

SimpleObstacleController::SimpleObstacleController(ros::NodeHandle *n,
                                       ros::Publisher *command_pub)
    : n(n), command_pub(command_pub){
  ROS_INFO("New SimpleObstacleController");

  laser_sub = n->subscribe<sensor_msgs::LaserScan>(
      "/scan", 10, &SimpleObstacleController::getCurrentLaserScan, this);

  sensor_sub = n->subscribe<pses_basis::SensorData>(
      "pses_basis/sensor_data", 10, &SimpleObstacleController::getCurrentSensorData,
      this);

  initialized = false;
}

SimpleObstacleController::~SimpleObstacleController() {
  ROS_INFO("Destroying SimpleObstacleController");
  laser_sub.shutdown();
}

void SimpleObstacleController::updateDistanceToObstacle() {  
  float d_min = std::numeric_limits<float>::max();
  float d_max = std::numeric_limits<float>::min();
  float alpha_min = 0;
  float alpha_max = 0;
  for (size_t i = 0; i < currentLaserScan->ranges.size(); ++i) {
    const float r = currentLaserScan->ranges[i];
    if (currentLaserScan->range_min < r && r < currentLaserScan->range_max) {
      // alpha in radians and always positive
      const float alpha =
          i * currentLaserScan->angle_increment + currentLaserScan->angle_min;
      const float sin_alpha = std::sin(std::abs(alpha));
      const float b = CAR_WIDTH / 2.0 + 0.4;
      if (sin_alpha != 0.0 && r < b / sin_alpha) {
        // distance to obstacle
        const float d = r * std::cos(std::abs(alpha));
        if (d < d_min){
          d_min = d;
          alpha_min = alpha;
        }
        if (d > d_max){
          d_max = d;
          alpha_max = alpha;
        }
      }
    }
  }
  // camera isn't at the car's front
  d_min -= 0.1;
  if (d_min < 0)
    d_min = 0;
  else if (d_min > currentLaserScan->range_max)
    d_min = currentLaserScan->range_max;
  
  obstacleDistace = d_min;
  if(obstacleDistace < 0.7){
    ROS_INFO("***** Obstacle ********");
    ROS_INFO("angle: [%f]", alpha_min);
    if(alpha_min < 0.0){
      currentHeadingAngle = 1.0;
    } else {
      currentHeadingAngle = -1.0;
    }
    //currentHeadingAngle = alpha_max + 6.0 * (alpha_max - alpha_min);    
  }
}

void SimpleObstacleController::getBestHeadingAngle() {
  float d_max = std::numeric_limits<float>::min();
  float alpha_max = 0;

  float distortStep = 1.0 / currentLaserScan->ranges.size(); 

  double rangeLeft = currentSensorData->range_sensor_left;
  rangeLeft = rangeLeft * 0.8;
  if(rangeLeft != 0.0){
    rangeLeft = std::min(rangeLeft, 2.5);
    rangeLeft = std::max(rangeLeft, 0.1);
    distortStep = ((float) rangeLeft / currentLaserScan->ranges.size());
    ROS_INFO("distort range: [%f]", (float) rangeLeft);
    ROS_INFO("distort: [%f]", distortStep);
  }


  for (size_t i = 0; i < currentLaserScan->ranges.size(); ++i) {
    const float r = currentLaserScan->ranges[i];
    if (currentLaserScan->range_min < r && r < currentLaserScan->range_max) {
      // alpha in radians and always positive
      const float alpha =
          i * currentLaserScan->angle_increment + currentLaserScan->angle_min;
      float d = r * std::cos(alpha) * (0.0005 + pow(i, 1.3) * distortStep);
      if (d > d_max){
          d_max = d;
          alpha_max = alpha;
        }
    }
  }

  currentHeadingAngle = alpha_max;
}

void SimpleObstacleController::simpleController(){
  this->updateDistanceToObstacle();
  if(obstacleDistace > 0.7){
    this->getBestHeadingAngle();    
  }
  //ROS_INFO("distance: [%f], angle: [%f]", obstacleDistace, currentHeadingAngle);

  pses_basis::Command cmd;
  int motorLevel = std::min((int)(obstacleDistace * 5.0), 20);
  int maxSpeedUS = (int) ((currentSensorData->range_sensor_left) * 12.0);
  motorLevel = std::min(motorLevel, maxSpeedUS);
  cmd.motor_level = std::max(motorLevel, 4);
  cmd.steering_level = (int)(currentHeadingAngle * 250.0);

  
  // If left distance is smaller than ....
  if(0.0 < currentSensorData->range_sensor_left && currentSensorData->range_sensor_left < 0.2){
    ROS_INFO("is parallel to wall");
    if(cmd.steering_level > 0){
      cmd.steering_level = 0;
    }
  }
  if(0.0 < currentSensorData->range_sensor_right && currentSensorData->range_sensor_right < 0.2 && cmd.steering_level < 0){
    cmd.steering_level = 0;
  }

  if (cmd.steering_level > 40)
    cmd.steering_level = 40;
  else if (cmd.steering_level < -40)
    cmd.steering_level = -40;

  ROS_INFO("steering_level: [%d]", cmd.steering_level);
  cmd.header.stamp = ros::Time::now();
  command_pub->publish(cmd);
  ros::spinOnce();
}

void SimpleObstacleController::getCurrentLaserScan(
    const sensor_msgs::LaserScan::ConstPtr &msg) {
  currentLaserScan = msg;
  //laserDetector->setCurrentLaserScan(msg);
}

void SimpleObstacleController::getCurrentSensorData(
    const pses_basis::SensorData::ConstPtr &msg) {
  currentSensorData = msg;
}

void SimpleObstacleController::run() {
  if (!initialized) {
    if (currentLaserScan == nullptr ||
        currentLaserScan->ranges[0] == -1.0) {
      ROS_INFO("Sensors Uninitialized!");
      return;
    } else {
      initialized = true;
      //laserDetector->initialize();
    }
  }

  // If everything is initialized, run Controller
  this->simpleController();
}

#endif
