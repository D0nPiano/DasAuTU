#ifndef CURVEDRIVER2_H
#define CURVEDRIVER2_H

#include "ros/ros.h"

#include "nav_msgs/Odometry.h"

class CurveDriver2 {
public:
  CurveDriver2(ros::NodeHandle &nh);
  void reset();
  void drive(const nav_msgs::OdometryConstPtr &odom);
  bool isAroundTheCorner() const;
  void curveInit(float radius, bool left,
                 const nav_msgs::OdometryConstPtr &odom);

private:
  float e0;
  double t0;
  float radius;
  int16_t initialSteering;
  float steerfact, steerfactAbs;
  ros::Publisher command_pub;
  geometry_msgs::Pose rotationCenter;
};

#endif // CURVEDRIVER2_H
