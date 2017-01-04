#ifndef EMERGENCYBRAKE_H
#define EMERGENCYBRAKE_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "pses_basis/CarInfo.h"
#include "pses_basis/Command.h"
#include "pses_basis/SensorData.h"

class EmergencyBrake {
public:
  EmergencyBrake(ros::NodeHandle *n);
  ~EmergencyBrake();
  void carInfoCallback(const pses_basis::CarInfoConstPtr &msg);
  void commandCallback(const pses_basis::CommandConstPtr &cmd);
  void sensorDataCallback(const pses_basis::SensorDataConstPtr &msg);
  void laserscanCallback(const sensor_msgs::LaserScanConstPtr &msg);
  void timerCallback(const ros::TimerEvent &);

private:
  void updateDistanceToObstacle();
  ros::Publisher command_pub;
  // ros::Publisher debug_pub;
  ros::Subscriber command_sub;
  ros::Subscriber sensor_sub;
  ros::Subscriber laserscan_sub;
  ros::Subscriber carinfo_sub;
  ros::Timer timer;
  float duration;
  int16_t maxMotorLevel;
  float currentSpeed;
  float speedCarInfo;
  double speedTimestamp;
  float us_front;
  float carWidth;
  float distanceToObstacle;
  float deceleration, safetyDistance;
  sensor_msgs::LaserScanConstPtr laserscan;
  uint8_t state;
  int16_t lastSteering;
};

#endif // EMERGENCYBRAKE_H
