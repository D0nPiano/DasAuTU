#pragma once

#include "autu_control/AutoController.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

class RundkursController : public AutoController  {
   public:
   		RundkursController(ros::NodeHandle * n);
    	void run();
   private:
   		void simpleController();
   		ros::NodeHandle * n;
};

RundkursController::RundkursController(ros::NodeHandle * n):
	n(n)
	{
	ROS_INFO("New RundkursController");
}

void RundkursController::simpleController(){
  ROS_INFO("test - ******************");
}

void RundkursController::run() {
	this->simpleController();
}
