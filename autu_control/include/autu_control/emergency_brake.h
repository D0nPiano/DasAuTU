#ifndef EMERGENCYBRAKE_H
#define EMERGENCYBRAKE_H

#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "pses_basis/CarInfo.h"
#include "pses_basis/Command.h"

class EmergencyBrake {
public:
  EmergencyBrake(ros::NodeHandle *n);
  ~EmergencyBrake();
  void commandCallback(const pses_basis::CommandConstPtr &cmd);
  void speedCallback(const pses_basis::CarInfoConstPtr &msg);
  void odomCallback(const nav_msgs::OdometryConstPtr &msg);
  void updateDistanceToObstacle();
  void laserscanCallback(const sensor_msgs::LaserScanConstPtr &msg);
  void timerCallback(const ros::TimerEvent &);

private:
  ros::Publisher command_pub;
  ros::Subscriber command_sub;
  ros::Subscriber speed_sub;
  ros::Subscriber odom_sub;
  ros::Subscriber laserscan_sub;
  ros::Timer timer;
  int16_t maxMotorLevel;
  float currentSpeed;
  float carWidth;
  float distanceToObstacle;
  sensor_msgs::LaserScanConstPtr laserscan;
};

#endif // EMERGENCYBRAKE_H
