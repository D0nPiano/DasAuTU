#pragma once

/*
	If LaserScan is uninitialized, it's range[0] is -1.0
	If SensorData is uninitialized, it's range_sensor_left is -1.0
*/


#include "autu_control/AutoController.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include "pses_basis/SensorData.h"
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <exception>

class RundkursController : public AutoController  {
   public:
   		RundkursController(ros::NodeHandle * n);
   		~RundkursController();
    	void run();
   private:
   		void simpleController();
   		void getCurrentLaserScan(const sensor_msgs::LaserScan::ConstPtr& );
   		void getCurrentSensorData(const pses_basis::SensorData::ConstPtr& );
   		ros::NodeHandle * n;
   		ros::Subscriber laser_sub;
   		ros::Subscriber sensor_sub;
   		sensor_msgs::LaserScan *currentLaserScan;
   		pses_basis::SensorData *currentSensorData;
};

RundkursController::RundkursController(ros::NodeHandle * n): n(n){
	ROS_INFO("New RundkursController");
	laser_sub = n->subscribe<sensor_msgs::LaserScan>(
      "/scan", 1000, &RundkursController::getCurrentLaserScan, this);

	sensor_sub = n->subscribe<pses_basis::SensorData>(
      "pses_basis/sensor_data", 1000, &RundkursController::getCurrentSensorData, this);


	// Initialite LaserScan
	currentLaserScan = new sensor_msgs::LaserScan;
	std::vector<float> tmpRanges;
	tmpRanges.push_back(-1.0);
	currentLaserScan->ranges = tmpRanges;

	// Initialize SensorData
	currentSensorData = new pses_basis::SensorData;
	currentSensorData->range_sensor_left = -1.0;
}

RundkursController::~RundkursController()
{
	ROS_INFO("Destroying RundkursController");
	laser_sub.shutdown();
	sensor_sub.shutdown();
}

void RundkursController::getCurrentLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg){
	*currentLaserScan = *msg;
}

void RundkursController::getCurrentSensorData(const pses_basis::SensorData::ConstPtr&  msg){
	*currentSensorData = *msg;
}

void RundkursController::simpleController(){
  ROS_INFO("test - ******************");
  ROS_INFO("Angle to wall in deg: [%f]", currentSensorData->range_sensor_left);
}

void RundkursController::run() {
	if(currentLaserScan->ranges[0] == -1.0 || currentSensorData->range_sensor_left == -1.0){
		ROS_INFO("Uninitialized!");
		return;
	}
	//If everything is initialized, run Controller
	this->simpleController();
}
