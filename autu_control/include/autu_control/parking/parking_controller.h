#ifndef PARKINGCONTROLLER_H
#define PARKINGCONTROLLER_H

#include "autu_control/AutoController.h"
#include "autu_control/rundkurs/laser_utilities.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class ParkingController : public AutoController {
public:
  ParkingController(ros::NodeHandle &nh);
  void odomCallback(const nav_msgs::OdometryConstPtr &msg);
  void laserscanCallback(const sensor_msgs::LaserScanConstPtr &msg);
  void run();

private:
  ros::Publisher command_pub;
  ros::Subscriber odom_sub;
  ros::Subscriber laser_sub;
  nav_msgs::OdometryConstPtr odom;
  sensor_msgs::LaserScanConstPtr laserscan;
  geometry_msgs::Pose curveBegin;
  LaserUtil laserUtil;
  uint8_t state;
  int16_t velocity;
  int16_t maxSteering;
  float maxAngle;
};

#endif // PARKINGCONTROLLER_H
