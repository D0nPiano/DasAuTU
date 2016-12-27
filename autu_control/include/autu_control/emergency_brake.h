#ifndef EMERGENCYBRAKE_H
#define EMERGENCYBRAKE_H

#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "pses_basis/CarInfo.h"
#include "pses_basis/Command.h"
#include "pses_basis/SensorData.h"

class EmergencyBrake {
public:
  EmergencyBrake(ros::NodeHandle *n);
  ~EmergencyBrake();
  void commandCallback(const pses_basis::CommandConstPtr &cmd);
  void speedCallback(const pses_basis::CarInfoConstPtr &msg);
  void sensorDataCallback(const pses_basis::SensorDataConstPtr &msg);
  void odomCallback(const nav_msgs::OdometryConstPtr &msg);
  void laserscanCallback(const sensor_msgs::LaserScanConstPtr &msg);
  void timerCallback(const ros::TimerEvent &);

private:
  void updateDistanceToObstacle();
  ros::Publisher command_pub;
  ros::Subscriber command_sub;
  ros::Subscriber speed_sub;
  ros::Subscriber sensor_sub;
  ros::Subscriber odom_sub;
  ros::Subscriber laserscan_sub;
  ros::Timer timer;
  int16_t maxMotorLevel;
  float currentSpeed;
  float us_front;
  float carWidth;
  float distanceToObstacle;
  float deceleration, safetyDistance;
  sensor_msgs::LaserScanConstPtr laserscan;
  uint8_t state;
};

#endif // EMERGENCYBRAKE_H
