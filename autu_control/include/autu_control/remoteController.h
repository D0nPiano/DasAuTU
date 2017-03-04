#pragma once

#include "autu_control/AutoController.h"
#include "ros/ros.h"
/**
 * @brief Controller for controlling the car remote with the dashboard or app
 *
 */
class RemoteController : public AutoController {
public:
  RemoteController(ros::NodeHandle *n, ros::Publisher *command_pub);
  ~RemoteController();
  void run();

private:
  ros::NodeHandle *n;
};

RemoteController::RemoteController(ros::NodeHandle *n,
                                   ros::Publisher *command_pub)
    : n(n) {
  ROS_INFO("New RemoteController");
}

void RemoteController::run() {}

RemoteController::~RemoteController() {}
