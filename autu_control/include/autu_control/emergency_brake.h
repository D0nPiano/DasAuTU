#ifndef EMERGENCYBRAKE_H
#define EMERGENCYBRAKE_H

#include <vector>

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include <tf/transform_listener.h>

#include "pses_basis/CarInfo.h"
#include "pses_basis/Command.h"

class EmergencyBrake {
public:
  EmergencyBrake(ros::NodeHandle *n);
  ~EmergencyBrake();
  void gridMapCallback(const nav_msgs::OccupancyGridConstPtr &msg);
  void commandCallback(const pses_basis::CommandConstPtr &cmd);
  void speedCallback(const pses_basis::CarInfoConstPtr &msg);
  void odomCallback(const nav_msgs::OdometryConstPtr &msg);
  void occupyCell(nav_msgs::OccupancyGrid &gridmap,
                  const geometry_msgs::Point &point);
  void timerCallback(const ros::TimerEvent &);

private:
  ros::Publisher command_pub;
  ros::Publisher map_pub;
  ros::Subscriber command_sub;
  ros::Subscriber speed_sub;
  ros::Subscriber map_sub;
  ros::Subscriber odom_sub;
  ros::Timer timer;
  int16_t maxMotorLevel;
  float currentSpeed;
  nav_msgs::OccupancyGridConstPtr grid;
  tf::TransformListener tfListener;
  uint32_t gridId;
  double carX, carY;
};

#endif // EMERGENCYBRAKE_H
