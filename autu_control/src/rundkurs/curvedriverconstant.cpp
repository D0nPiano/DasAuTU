#include "autu_control/rundkurs/curvedriverconstant.h"

#include "Eigen/Dense"
#include <cmath>
#include <limits>
#include <vector>

#include "pses_basis/Command.h"

#define WHEELBASE 0.28

using std::sqrt;

CurveDriverConstant::CurveDriverConstant(ros::NodeHandle &nh,
                                         LaserUtil &laserUtil)
    : scanOffset(0), laserUtil(laserUtil) {
  command_pub = nh.advertise<pses_basis::Command>("autu/command", 1);

  maxMotorLevel = nh.param<int>("main/curvedriver_constant/max_motor_level", 8);
  steering = nh.param<int>("main/curvedriver_constant/steering", 25);

  corner_threshold =
      nh.param<float>("main/curvedriver_constant/corner_threshold", 0.5f);
}

void CurveDriverConstant::reset() {}

void CurveDriverConstant::drive() {

  pses_basis::Command cmd;

  cmd.motor_level = maxMotorLevel;
  cmd.steering_level = steering;

  cmd.header.stamp = ros::Time::now();
  command_pub.publish(cmd);
}

bool CurveDriverConstant::isAroundTheCorner() const {
  tf::Quaternion start, current;
  tf::quaternionMsgToTF(curveBegin.orientation, start);
  tf::quaternionMsgToTF(odom->pose.pose.orientation, current);

  return start.angle(current) > 78 * M_PI / 180.0 / 2;
}

void CurveDriverConstant::curveInit(float radius, bool left) {
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
}

bool CurveDriverConstant::isNextToCorner(bool left, float speed) {
  if (laserscan == nullptr)
    return false;
  updateScanOffset(speed);
  Eigen::Vector2f vecToCorner;
  float last_r = std::numeric_limits<float>::max();
  size_t cornerIndex = 0;
  if (left) {
    size_t i;
    for (i = laserscan->ranges.size() - 1; i > laserscan->ranges.size() / 2;
         --i) {
      const float r = laserscan->ranges[i];
      if (laserscan->range_min < r && r < laserscan->range_max) {
        if (r - last_r > corner_threshold)
          break;
        else
          last_r = r;
      }
    }
    if (last_r == std::numeric_limits<float>::max())
      return false;
    cornerIndex = i + 1;
    const float alpha =
        std::abs(laserscan->angle_min + (i + 1) * laserscan->angle_increment);
    corner.x = last_r * std::cos(alpha) - scanOffset;
    corner.y = last_r * std::sin(alpha);
    vecToCorner[0] = last_r * std::cos(alpha);
    vecToCorner[1] = last_r * std::sin(alpha);
  }
  cornerSeen = odom->pose.pose;

  const float vc = maxMotorLevel / 10.0f;
  const float dif = vc - speed;
  distance_to_corner = dif * dif / -2 + speed * (speed - vc);
  return corner.x - 0.1f < 0.8f + distance_to_corner;
  // laserUtil.calcCornerSize(laserscan, vecToCorner, left) > 1.2f;
}

bool CurveDriverConstant::isAtCurveBegin(bool left) const {
  const float xDif = cornerSeen.position.x - odom->pose.pose.position.x;
  const float yDif = cornerSeen.position.y - odom->pose.pose.position.y;

  // actual distance
  const float distance = sqrt(xDif * xDif + yDif * yDif);

  return distance > distance_to_corner;
}

void CurveDriverConstant::setLaserscan(
    const sensor_msgs::LaserScanConstPtr &scan) {
  if (scan == nullptr)
    return;
  if (laserscan == nullptr || scan != laserscan) {
    scanOffset = 0;
    scanOffsetStamp = scan->header.stamp.toSec();
    laserscan = scan;
  }
}

void CurveDriverConstant::setOdom(const nav_msgs::OdometryConstPtr &msg) {
  odom = msg;
}

void CurveDriverConstant::updateScanOffset(float speed) {
  const double now = ros::Time::now().toSec();
  const double timeDif = now - scanOffsetStamp;
  scanOffset += timeDif * speed;
  scanOffsetStamp = now;
}
