#pragma once

#include "autu_control/AutoController.h"
#include "ros/ros.h"

class RemoteController : public AutoController  {
   public:
   		RemoteController(ros::NodeHandle * n);
    	void run();
   private:
   		ros::NodeHandle * n;
};


RemoteController::RemoteController(ros::NodeHandle * n):
  n(n)
  {
  ROS_INFO("New RemoteController");
}

void RemoteController::run() {
	ROS_INFO("Remote");
}
