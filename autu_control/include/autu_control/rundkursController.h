#pragma once

/*
	If LaserScan is uninitialized, it's range[0] is -1.0
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
   		ros::NodeHandle * n;
   		ros::Subscriber laser_sub;
   		sensor_msgs::LaserScan *current_scan;
};

RundkursController::RundkursController(ros::NodeHandle * n): n(n){
	ROS_INFO("New RundkursController");
	laser_sub = n->subscribe<sensor_msgs::LaserScan>(
      "/scan", 1000, &RundkursController::getCurrentLaserScan, this);
	current_scan = new sensor_msgs::LaserScan;


	std::vector<float> tmpRanges;
	tmpRanges.push_back(-1.0);
	current_scan->ranges = tmpRanges;
}

RundkursController::~RundkursController()
{
	ROS_INFO("Destroying RundkursController");
	laser_sub.shutdown();
}

void RundkursController::getCurrentLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg){
	*current_scan = *msg;
}

void RundkursController::simpleController(){
  ROS_INFO("test - ******************");
  ROS_INFO("Angle to wall in deg: [%f]", current_scan->ranges[0]);
}

void RundkursController::run() {
	if(current_scan->ranges[0] == -1){
		ROS_INFO("Uninitialized!");
		return;
	}
	this->simpleController();
}
