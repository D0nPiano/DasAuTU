#include "autu_control/rundkurs/curvedriver2.h"

#include <math.h>

#include "pses_basis/Command.h"

#define MIN_RADIUS 0.6
#define MAX_STEERING_ANGLE 20.0
#define MAX_STEERING_LEVEL 40.0
#define WHEELBASE 0.28

CurveDriver2::CurveDriver2(ros::NodeHandle &nh)
    : e0(0), t0(0), radius(1), steerfactAbs(2) {
  command_pub = nh.advertise<pses_basis::Command>("autu/command", 1);
}

void CurveDriver2::reset() {
  e0 = 0;
  t0 = 0;
}

void CurveDriver2::drive(const nav_msgs::OdometryConstPtr &odom) {
  const float xDif = rotationCenter.position.x - odom->pose.pose.position.x;
  const float yDif = rotationCenter.position.y - odom->pose.pose.position.y;
  // actual distance
  const float currentRadius = std::sqrt(xDif * xDif + yDif * yDif);

  pses_basis::Command cmd;

  // P-Regler, tb = 62s

  cmd.motor_level = 8;
  float p = 16;
  float d = 8;
  double t = ros::Time::now().toSec();
  float e = radius - currentRadius;
  cmd.steering_level =
      initialSteering + steerfact * p * (e + (e - e0) * d / (t - t0));

  e0 = e;
  t0 = t;

  if (cmd.steering_level > 40)
    cmd.steering_level = 40;
  else if (cmd.steering_level < -40)
    cmd.steering_level = -40;

  cmd.header.stamp = ros::Time::now();
  command_pub.publish(cmd);
}

bool CurveDriver2::isAroundTheCorner() const { return false; }

void CurveDriver2::curveInit(float radius, bool left,
                             const nav_msgs::OdometryConstPtr &odom) {
  this->radius = radius;
  const std::string targetFrame(left ? "/rear_left_wheel"
                                     : "/rear_right_wheel");
  rotationCenter.position = odom->pose.pose.position;
  try {
    transformListener.waitForTransform(targetFrame, "/odom", ros::Time(0),
                                       ros::Duration(10.0));
    transformListener.lookupTransform(targetFrame, "/odom", ros::Time(0),
                                      transform);

    geometry_msgs::PointStamped pointICC, pointOdom;

    if (left)
      pointICC.point.y += radius;
    else
      pointICC.point.y -= radius;

    pointICC.header.frame_id = targetFrame;
    pointICC.header.stamp = ros::Time(0);
    transformListener.transformPoint("/odom", pointICC, pointOdom);
    rotationCenter.position.x += pointOdom.point.x;
    rotationCenter.position.y += pointOdom.point.y;
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }

  ROS_INFO("odomPoint x: %f y: %f", odom->pose.pose.position.x,
           odom->pose.pose.position.y);
  ROS_INFO("wheelPoint x: %f y: %f", rotationCenter.position.x,
           rotationCenter.position.y);

  /*  if (left)
      rotationCenter.position.y += radius;
    else
      rotationCenter.position.y -= radius;*/

  // alpha in radians
  const float alpha = M_PI / 2 - std::atan(radius / WHEELBASE);
  initialSteering =
      alpha * 180 / M_PI / MAX_STEERING_ANGLE * MAX_STEERING_LEVEL;

  if (left)
    steerfact = -steerfactAbs;
  else {
    initialSteering = -initialSteering;
    steerfact = steerfactAbs;
  }
}
