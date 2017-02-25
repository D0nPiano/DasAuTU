#ifndef _SimpleObstacleController_CPP_
#define _SimpleObstacleController_CPP_

#include "autu_control/simpleObstacle/simpleObstacleController.h"
#include "ros/ros.h"
#include <math.h>       /* pow */

#define CAR_WIDTH 0.2		// Car width in m
//#define minWallDist 0.4	// Wall distance in m
//#define steeringMulti 0.3	// between 0.0 (straight) and 1.0 (full)
//#define distortPow 1.2		// behave linear (1) or quadradic (2.0)-> Higher: Drive further left
//#define distortUSInfluencePow 1.5
//#define obstacleSteeringPow 0.
#define OBSTACLE_DIST 0.5   // obstacle Distance fo emergency reaction


SimpleObstacleController::SimpleObstacleController(ros::NodeHandle *n,
                                       ros::Publisher *command_pub)
    : n(n), command_pub(command_pub), lowpass(5){
  ROS_INFO("New SimpleObstacleController");

  laser_sub = n->subscribe<sensor_msgs::LaserScan>(
      "/scan", 10, &SimpleObstacleController::getCurrentLaserScan, this);

  sensor_sub = n->subscribe<pses_basis::SensorData>(
      "pses_basis/sensor_data", 10, &SimpleObstacleController::getCurrentSensorData,
      this);



  initialized = false;

  minWallDist = n->param<float>("main/obstacleController/minWallDist", 0.2);
  steeringMulti = n->param<float>("main/obstacleController/steeringMulti", 0.4);
  distortPow = n->param<float>("main/obstacleController/distortPow", 1.2);
  distortUSInfluencePow = n->param<float>("main/obstacleController/distortUSInfluencePow", 1.5);
  obstacleSteeringPow = n->param<float>("main/obstacleController/obstacleSteeringPow", 0.6);
  PIDMotorLevel = n->param<int>("main/obstacleController/PIDMotorLevel", 5);
  PIDWallDistance = n->param<float>("main/obstacleController/PIDWallDistance", 0.4);
  PIDP = n->param<float>("main/obstacleController/PIDP", 10.0);
  PIDD = n->param<float>("main/obstacleController/PIDD", 1.0);
  obstacleMotorLevel = n->param<float>("main/obstacleController/obstacleMotorLevel", 0.8);

    pidRegler = new PIDRegler(*n, PIDP, PIDD, PIDMotorLevel, PIDWallDistance);


  ROS_INFO_STREAM("Alle Werte: " << minWallDist << steeringMulti << distortPow << distortUSInfluencePow << obstacleSteeringPow);
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
      const float b = CAR_WIDTH / 2.0 + minWallDist;
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
  
  if(obstacleDistace < OBSTACLE_DIST){

    ROS_INFO("***** Obstacle ********");
    //obstacleDistace = pow(obstacleDistace, obstacleSteeringPow);
  // sum pu left side
    double sumLeft, sumRight;
    for (size_t i = 0; i < (int)(currentLaserScan->ranges.size() / 2); ++i) {
      const float r = currentLaserScan->ranges[i];
      if (currentLaserScan->range_min < r && r < currentLaserScan->range_max) {
        // alpha in radians and always positive
        const float alpha =
            i * currentLaserScan->angle_increment + currentLaserScan->angle_min;
        sumRight += r * std::cos(std::abs(alpha));
      }
    }

    for (size_t i = (int)(currentLaserScan->ranges.size() / 2); i < currentLaserScan->ranges.size(); ++i) {
      const float r = currentLaserScan->ranges[i];
      if (currentLaserScan->range_min < r && r < currentLaserScan->range_max) {
        // alpha in radians and always positive
        const float alpha =
            i * currentLaserScan->angle_increment + currentLaserScan->angle_min;
        sumLeft += r * std::cos(std::abs(alpha));
      }
    }


  ROS_INFO("Right [%f]", sumRight);
  ROS_INFO("Left: [%f]", sumLeft);

    ROS_INFO("angle: [%f]", alpha_min);
    float distanceFactor = obstacleSteeringPow/obstacleDistace ;
    if(sumRight < sumLeft){
      currentHeadingAngle = distanceFactor * 90.0 / (250.0 * steeringMulti);
    } else {
      currentHeadingAngle = distanceFactor * -90.0 / (250.0 * steeringMulti);
    }
    //currentHeadingAngle = alpha_max + 6.0 * (alpha_max - alpha_min);    
  }

  ROS_INFO("Obstacle: [%f]", obstacleDistace);
}

float SimpleObstacleController::getWrackingDistance(){
  float d_min = std::numeric_limits<float>::max();
  for (size_t i = 0; i < currentLaserScan->ranges.size(); ++i) {
    const float r = currentLaserScan->ranges[i];
    if (currentLaserScan->range_min < r && r < currentLaserScan->range_max) {
      // alpha in radians and always positive
      const float alpha =
          i * currentLaserScan->angle_increment + currentLaserScan->angle_min;
      const float sin_alpha = std::sin(std::abs(alpha));
      const float b = CAR_WIDTH / 2.0;
      if (sin_alpha != 0.0 && r < b / sin_alpha) {
        // distance to obstacle
        const float d = r * std::cos(std::abs(alpha));
        if (d < d_min){
          d_min = d;
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

  ROS_INFO("Wracking: [%f]", d_min);
  return d_min;
}

int SimpleObstacleController::getBestSpeed(){
  //int motorLevel = std::min((int)(this->getWrackingDistance() * obstacleMotorLevel), 20);
  float distanceAverage = (obstacleDistace + 0.5 * this->getWrackingDistance()) / 1.5;
  int motorLevel = (int)(distanceAverage * obstacleMotorLevel);
  //int maxSpeedUS = (int) ((currentSensorData->range_sensor_left) * 30.0);
 // motorLevel = std::min(motorLevel, maxSpeedUS);
  // int steering_level = this->getBestSteering();
  //motorLevel =  3*motorLevel/(std::abs((steering_level) / 8 + 1));
 // ROS_INFO("steering_level: [%d]", steering_level);
  ROS_INFO("motor_level: [%d]", motorLevel);
  return std::min(12,std::max(motorLevel, 7));
}

int SimpleObstacleController::getBestSteering(){
  int steering_level = (int)(currentHeadingAngle * 250.0 * steeringMulti);

  
  // If left distance is smaller than ....
  if(lowpass.getAverage() < 0.3){
    if(steering_level > 0){
      ROS_INFO("is next to wall - not steering left");
      steering_level = 0;
    }
  }
  if(0.0 < currentSensorData->range_sensor_right && currentSensorData->range_sensor_right < 0.3 && steering_level < 0){
    ROS_INFO("is next to wall - not steering right");
    steering_level = 0;
  }

  
  ROS_INFO("steering_level: [%d]", steering_level);

  if (steering_level > 40)
    steering_level = 40;
  else if (steering_level < -40)
    steering_level = -40;

  //ROS_INFO("steering_level: [%d]", steering_level);
  return steering_level;
}

void SimpleObstacleController::getBestHeadingAngle() {
  float d_max = std::numeric_limits<float>::min();
  float alpha_max = 0;

  float distortStep = 1.0 / currentLaserScan->ranges.size(); 

  double rangeLeft = lowpass.getAverage();
  if(rangeLeft != 0.0){
    rangeLeft = std::min(rangeLeft, 2.5);
    rangeLeft = std::max(rangeLeft, 0.1);
  	rangeLeft = pow (rangeLeft, distortUSInfluencePow) / pow(2.5, distortUSInfluencePow);
    distortStep = ((float) rangeLeft / currentLaserScan->ranges.size());
    //ROS_INFO("distort range: [%f]", (float) rangeLeft);
    //ROS_INFO("distort: [%f]", distortStep);
  }


  for (size_t i = 0; i < currentLaserScan->ranges.size(); ++i) {
    float r = currentLaserScan->ranges[i];
    if (currentLaserScan->range_min < r && r < currentLaserScan->range_max) {
    	if(r > 6.0){
    		r = 6.0;
    	}
      // alpha in radians and always positive
      const float alpha =
          i * currentLaserScan->angle_increment + currentLaserScan->angle_min;
      float d = r * std::cos(alpha) * (0.001 + pow(i, distortPow) * distortStep);
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
  if(obstacleDistace > OBSTACLE_DIST){ // does not have to avoid obstacle immediatly
    this->getBestHeadingAngle();
    int steering_level = this->getBestSteering();
    if(lowpass.getAverage() < 1.5 && obstacleDistace > 1.2){ // is next to Wall and wants to drive left
	    // use PID-Regler
	    ROS_INFO("using PID Controller");
      	pidRegler->setMaxMotorLevel(std::min(this->getBestSpeed(),PIDMotorLevel));
		pidRegler->drive(lowpass.getAverage(), true);
	    return;
	}
  }
  //ROS_INFO("distance: [%f], angle: [%f]", obstacleDistace, currentHeadingAngle);

  pses_basis::Command cmd;
  cmd.motor_level = this->getBestSpeed();
  cmd.steering_level = this->getBestSteering();
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

  if(msg->range_sensor_left >0){
    lowpass.addValue(msg->range_sensor_left);
  }
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
