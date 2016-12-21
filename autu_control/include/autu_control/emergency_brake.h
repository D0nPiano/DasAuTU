#ifndef EMERGENCYBRAKE_H
#define EMERGENCYBRAKE_H

#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"

#include "pses_basis/Command.h"

class EmergencyBrake {
public:
  EmergencyBrake(ros::NodeHandle *n);
  void gridMapCallback(const nav_msgs::OccupancyGridConstPtr &msg);
  void commandCallback(const pses_basis::CommandConstPtr &cmd);

private:
  ros::Publisher command_pub;
  int16_t maxMotorLevel;
};

#endif // EMERGENCYBRAKE_H
