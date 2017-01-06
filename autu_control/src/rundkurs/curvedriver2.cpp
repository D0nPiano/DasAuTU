#include "autu_control/rundkurs/curvedriver2.h"

#include <cmath>
#include <limits>
#include <vector>

#include "pses_basis/Command.h"

#define MIN_RADIUS 0.6
#define MAX_STEERING_ANGLE 20.0
#define MAX_STEERING_LEVEL 40.0
#define WHEELBASE 0.28

using std::pair;
using std::sqrt;

CurveDriver2::CurveDriver2(ros::NodeHandle &nh)
    : e0(0), t0(0), radius(1), steerfactAbs(2), scanOffset(0) {
  command_pub = nh.advertise<pses_basis::Command>("autu/command", 1);

  maxMotorLevel = nh.param<int>("main/curvedriver/max_motor_level", 8);
}

void CurveDriver2::reset() {
  e0 = 0;
  t0 = 0;
}

void CurveDriver2::drive() {
  const float xDif = rotationCenter.position.x - odom->pose.pose.position.x;
  const float yDif = rotationCenter.position.y - odom->pose.pose.position.y;
  // actual distance
  const float currentRadius = sqrt(xDif * xDif + yDif * yDif);

  pses_basis::Command cmd;

  // P-Regler, tb = 62s

  cmd.motor_level = maxMotorLevel;
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

bool CurveDriver2::isAroundTheCorner() const {
  tf::Vector3 start, current;
  start.setX(curveBegin.position.x - rotationCenter.position.x);
  start.setY(curveBegin.position.y - rotationCenter.position.y);

  current.setX(odom->pose.pose.position.x - rotationCenter.position.x);
  current.setY(odom->pose.pose.position.y - rotationCenter.position.y);

  return start.angle(current) > M_PI_2;
}

void CurveDriver2::curveInit(float radius, bool left) {
  this->radius = radius;
  curveBegin = odom->pose.pose;
  const std::string targetFrame(left ? "/rear_left_wheel"
                                     : "/rear_right_wheel");

  try {
    transformListener.waitForTransform(targetFrame, "/odom", ros::Time(0),
                                       ros::Duration(0.1));
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
    rotationCenter.position.x = pointOdom.point.x;
    rotationCenter.position.y = pointOdom.point.y;
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }

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

bool CurveDriver2::isNextToCorner(bool left, float speed) {
  if (laserscan == nullptr)
    return false;
  updateScanOffset(speed);
  float last_r = std::numeric_limits<float>::max();

  if (left) {
    size_t i;
    for (i = laserscan->ranges.size() - 1; i > laserscan->ranges.size() / 2;
         --i) {
      const float r = laserscan->ranges[i];
      if (laserscan->range_min < r && r < laserscan->range_max) {
        if (r - last_r > 1.0)
          break;
        else
          last_r = r;
      }
    }
    if (last_r == std::numeric_limits<float>::max())
      return false;
    const float alpha =
        std::abs(laserscan->angle_min + (i + 1) * laserscan->angle_increment);
    corner.x = last_r * std::cos(alpha) - scanOffset;
    corner.y = last_r * std::sin(alpha);
  }
  cornerSeen = odom->pose.pose;
  cornerSeen.position.x += scanOffset;

  ROS_INFO("ScanOffset: %f Distance to corner: %f", scanOffset, corner.x);

  const float vc = maxMotorLevel / 10.0f;
  const float dif = vc - speed;
  distance_to_corner = dif * dif / -2 + speed * (speed - vc);
  // ROS_INFO("distance to corner: %f", distance_to_corner);
  return corner.x < 0.8f + 0.5f; // distance_to_corner;
}

bool CurveDriver2::isAtCurveBegin(bool left) const {
  const float xDif = cornerSeen.position.x - odom->pose.pose.position.x;
  const float yDif = cornerSeen.position.y - odom->pose.pose.position.y;

  // actual distance
  const float distance = sqrt(xDif * xDif + yDif * yDif);

  return distance > 0.5f; // distance_to_corner;
}

void CurveDriver2::setLaserscan(const sensor_msgs::LaserScanConstPtr &scan) {
  if (scan == nullptr)
    return;
  if (laserscan == nullptr || scan != laserscan) {
    scanOffset = 0;
    scanOffsetStamp = scan->header.stamp.toSec();
    laserscan = scan;
  }
}

void CurveDriver2::setOdom(const nav_msgs::OdometryConstPtr &msg) {
  odom = msg;
}

void CurveDriver2::updateScanOffset(float speed) {
  const double now = ros::Time::now().toSec();
  const double timeDif = now - scanOffsetStamp;
  // ROS_INFO("TimeDif: %f", timeDif);
  scanOffset += timeDif * speed;
  scanOffsetStamp = now;
}
