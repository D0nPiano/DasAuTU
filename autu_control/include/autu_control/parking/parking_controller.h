#ifndef PARKINGCONTROLLER_H
#define PARKINGCONTROLLER_H

#include "autu_control/AutoController.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"

class ParkingController : public AutoController {
public:
  ParkingController(ros::NodeHandle &nh);
  void odomCallback(const nav_msgs::OdometryConstPtr &msg);
  void run();

private:
  ros::Publisher command_pub;
  ros::Subscriber odom_sub;
  nav_msgs::OdometryConstPtr odom;
  geometry_msgs::Pose curveBegin;
  uint8_t state;
  int16_t velocity;
  int16_t maxSteering;
  float maxAngle;
};

#endif // PARKINGCONTROLLER_H
