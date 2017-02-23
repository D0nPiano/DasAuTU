#ifndef CURVEDRIVERCONSTANT_H
#define CURVEDRIVERCONSTANT_H

#include "autu_control/rundkurs/laser_utilities.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

class CurveDriverConstant {
public:
  CurveDriverConstant(ros::NodeHandle &nh, LaserUtil &laserUtil);
  void reset();
  void drive();
  bool isAroundTheCorner() const;
  void curveInit();
  bool isNextToCorner(float distanceToWall, float speed);
  bool rolloutBegins() const;
  bool isAtCurveBegin() const;
  void setLaserscan(const sensor_msgs::LaserScanConstPtr &scan);
  void setOdom(const nav_msgs::OdometryConstPtr &msg);

private:
  void updateScanOffset(float speed);
  bool isNextToGlas(float cornerX, float cornerY);
  bool wallFound(float cornerX, float cornerY);
  int16_t steering;
  int16_t maxMotorLevel;
  float corner_threshold;
  float rollout_distance;
  float blindness_offset;
  float scanOffset;
  double scanOffsetStamp;
  float corner_end_angle;
  float precurve_distance;
  float falseCornerDistance;
  float falseCornerEnd;
  float radius;
  float cornerSafetyDistance;
  float motorLevelFactor;
  bool falseCornerDetected;
  ros::Publisher command_pub;
  ros::Publisher corner_pub;
  ros::Publisher info_pub;
  geometry_msgs::Pose cornerSeen;
  geometry_msgs::Pose falseCorner, falseCornerSeen;
  geometry_msgs::Pose curveBegin;
  geometry_msgs::PointStamped cornerInOdom;
  tf::TransformListener transformListener;
  tf::StampedTransform transform;
  sensor_msgs::LaserScanConstPtr laserscan;
  nav_msgs::OdometryConstPtr odom;

  LaserUtil &laserUtil;
  struct Point {
    float x;
    float y;
  } corner;
};

#endif // CURVEDRIVERCONSTANT_H
