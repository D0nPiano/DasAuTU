#ifndef _ObstacleController_CPP_
#define _ObstacleController_CPP_

#include "autu_control/obstacles/obstacleController.h"
#include "ros/ros.h"

ObstacleController::ObstacleController(ros::NodeHandle *n,
                                       ros::Publisher *command_pub)
    : n(n), command_pub(command_pub) {
  ROS_INFO("New ObstacleController");

  plan_command_sub = n->subscribe<geometry_msgs::Twist>(
      "cmd_vel", 10, &ObstacleController::convertCommand, this);
}

ObstacleController::~ObstacleController() {
  ROS_INFO("Destroying ObstacleController");
  plan_command_sub.shutdown();
}

void ObstacleController::convertCommand(const geometry_msgs::Twist::ConstPtr& motionIn){
  command_data cmd;
  cmd.motor_level = int(motionIn->linear.x * 10);
  if(motionIn->linear.x > 0.0 && cmd.motor_level < 4){
  	cmd.motor_level = 4;
  } else if (motionIn->linear.x < 0.0 && cmd.motor_level > -10){
  	cmd.motor_level = -10;  	
  }
  
  cmd.steering_level = int((motionIn->angular.z) * 50);
  if(cmd.steering_level > 45){
  	cmd.steering_level = 45;
  } else if (cmd.steering_level < -45){
  	cmd.steering_level = -45;
  }

  cmd.header.stamp = ros::Time::now();
  command_pub->publish(cmd);
  ros::spinOnce();
}

void ObstacleController::run() {
}

#endif
