#ifndef CURVEDRIVER2_H
#define CURVEDRIVER2_H

#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

class CurveDriver2 {
public:
  CurveDriver2(ros::NodeHandle &nh);
  void reset();
  void drive(const nav_msgs::OdometryConstPtr &odom);
  bool isAroundTheCorner() const;
  void curveInit(float radius, bool left);
  bool isNextToCorner(bool left, float &cornerX);

  void setLaserscan(const sensor_msgs::LaserScanConstPtr &scan);

private:
  float e0;
  double t0;
  float radius;
  int16_t initialSteering;
  float steerfact, steerfactAbs;
  ros::Publisher command_pub;
  geometry_msgs::Pose rotationCenter;
  tf::TransformListener transformListener;
  tf::StampedTransform transform;
  sensor_msgs::LaserScanConstPtr laserscan;
  struct Point {
    float x;
    float y;
  } corner;
};

#endif // CURVEDRIVER2_H
