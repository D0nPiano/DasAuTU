#ifndef _SimpleObstacleController_H_
#define _SimpleObstacleController_H_

#define PI 3.14159265

/*
        If LaserScan is uninitialized, its range[0] is -1.0
        If SensorData is uninitialized, its range_sensor_left is -1.0
*/

#include "autu_control/AutoController.h"

#include "std_msgs/String.h"
#include <exception>
#include <iostream>
#include <memory>

#include "nav_msgs/Odometry.h"
#include <sensor_msgs/LaserScan.h>

#include "pses_basis/CarInfo.h"
#include "pses_basis/SensorData.h"
#include "ros/ros.h"
#include <pses_basis/Command.h>
typedef pses_basis::Command command_data;

class SimpleObstacleController : public AutoController {
public:
  SimpleObstacleController(ros::NodeHandle *n, ros::Publisher *command_pub);
  ~SimpleObstacleController();
  void getCurrentLaserScan(const sensor_msgs::LaserScan::ConstPtr &);
  void run();
private:
  ros::NodeHandle *n;
  ros::Publisher *command_pub;
  ros::Subscriber laser_sub;
  sensor_msgs::LaserScanConstPtr currentLaserScan;
  bool initialized;
};

#endif
