#ifndef CURVEDRIVERCONSTANT_H
#define CURVEDRIVERCONSTANT_H

#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

class CurveDriverConstant {
public:
  CurveDriverConstant(ros::NodeHandle &nh);
  void reset();
  void drive();
  bool isAroundTheCorner() const;
  void curveInit(float radius, bool left);
  bool isNextToCorner(bool left, float speed);
  bool isAtCurveBegin(bool left) const;
  void setLaserscan(const sensor_msgs::LaserScanConstPtr &scan);
  void setOdom(const nav_msgs::OdometryConstPtr &msg);

private:
  void updateScanOffset(float speed);

  int16_t steering;
  int16_t maxMotorLevel;
  float distance_to_corner;
  float scanOffset;
  double scanOffsetStamp;
  ros::Publisher command_pub;
  geometry_msgs::Pose cornerSeen;
  geometry_msgs::Pose curveBegin;
  geometry_msgs::Pose rotationCenter;
  tf::TransformListener transformListener;
  tf::StampedTransform transform;
  sensor_msgs::LaserScanConstPtr laserscan;
  nav_msgs::OdometryConstPtr odom;
  struct Point {
    float x;
    float y;
  } corner;
};

#endif // CURVEDRIVERCONSTANT_H
