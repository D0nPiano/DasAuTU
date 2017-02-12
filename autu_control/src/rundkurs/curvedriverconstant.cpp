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
    : scanOffset(0), falseCornerDetected(false), laserUtil(laserUtil) {
  command_pub = nh.advertise<pses_basis::Command>("autu/command", 1);

  maxMotorLevel = nh.param<int>("main/curvedriver_constant/max_motor_level", 8);
  steering = nh.param<int>("main/curvedriver_constant/steering", 25);

  corner_threshold =
      nh.param<float>("main/curvedriver_constant/corner_threshold", 0.5f);
  corner_end_angle =
      nh.param<float>("main/curvedriver_constant/corner_end_angle", 80.0f);
  precurve_distance =
      nh.param<float>("main/curvedriver_constant/precurve_distance", 0.8f);
  falseCornerEnd =
      nh.param<float>("main/curvedriver_constant/false_corner_end", 0.6f);
}

void CurveDriverConstant::reset() {}

void CurveDriverConstant::drive() {

  pses_basis::Command cmd;

  cmd.motor_level = maxMotorLevel;
  cmd.steering_level = steering;

  cmd.header.stamp = ros::Time::now();
  command_pub.publish(cmd);
}

bool CurveDriverConstant::isAroundTheCorner(
    const sensor_msgs::LaserScanConstPtr &scan) const {
  tf::Quaternion start, current;
  tf::quaternionMsgToTF(curveBegin.orientation, start);
  tf::quaternionMsgToTF(odom->pose.pose.orientation, current);

  return start.angle(current) > corner_end_angle * M_PI_2 / 180.0f; // &&
  // std::abs(laserUtil.getAngleToWallRLF(scan, true)) < 10 * M_PI / 180.0;
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

  if (falseCornerDetected) {
    const float xDif1 = falseCornerSeen.position.x - odom->pose.pose.position.x;
    const float yDif1 = falseCornerSeen.position.y - odom->pose.pose.position.y;

    // distance to falseCornerSeen
    const float distanceToCornerSeen = sqrt(xDif1 * xDif1 + yDif1 * yDif1);
    if (distanceToCornerSeen < falseCornerDistance)
      return false;

    const float xDif = falseCorner.position.x - odom->pose.pose.position.x;
    const float yDif = falseCorner.position.y - odom->pose.pose.position.y;

    // distance to falseCorner
    const float distanceToCorner = sqrt(xDif * xDif + yDif * yDif);
    if (distanceToCorner > falseCornerEnd) {
      ROS_INFO("********************** Glasscheibe zu Ende  "
               "**************************");
      falseCornerDetected = false;
    } else {
      ROS_INFO(
          "********************** Glas erkannt  **************************");
      return false;
    }
  }

  Eigen::Vector2f vecToCorner;
  float last_r = std::numeric_limits<float>::max();
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
    // if (i == laserscan->ranges.size() / 2)
    // return false;
    const float alpha =
        std::abs(laserscan->angle_min + (i + 1) * laserscan->angle_increment);
    corner.x = last_r * std::cos(alpha) - scanOffset;
    corner.y = last_r * std::sin(alpha);
    if (corner.x > 4.0f)
      return false;
    vecToCorner[0] = last_r * std::cos(alpha);
    vecToCorner[1] = last_r * std::sin(alpha);
  }
  cornerSeen = odom->pose.pose;

  const float vc = maxMotorLevel / 10.0f;
  const float dif = vc - speed;
  distance_to_corner = dif * dif / -2 + speed * (speed - vc);
  // ROS_INFO("Distance to corner max: %f", distance_to_corner);
  if (corner.x > 1.2f) {
    const float cornerSize =
        laserUtil.calcCornerSize(laserscan, vecToCorner, left);
    ROS_INFO("Corner Size: %f Corner.x: %f", cornerSize, corner.x);
    if (0 < cornerSize && cornerSize < 1.2f)
      try {
        tf::StampedTransform transform;
        transformListener.waitForTransform("base_laser", "/odom", ros::Time(0),
                                           ros::Duration(0.1));
        transformListener.lookupTransform("base_laser", "/odom", ros::Time(0),
                                          transform);

        geometry_msgs::PointStamped cornerInBaseLaser, cornerInOdom;

        cornerInBaseLaser.point.x = corner.x;
        cornerInBaseLaser.point.y = corner.y;
        cornerInBaseLaser.header.frame_id = "/base_laser";
        cornerInBaseLaser.header.stamp = ros::Time(0);
        transformListener.transformPoint("/odom", cornerInBaseLaser,
                                         cornerInOdom);

        falseCorner.position.x = cornerInOdom.point.x;
        falseCorner.position.y = cornerInOdom.point.y;

        falseCornerSeen = odom->pose.pose;
        falseCornerDistance = corner.x;
        falseCornerDetected = true;
        ROS_INFO("*********************False Corner "
                 "detected*******************************");
      } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        return false;
      }
  }

  return !falseCornerDetected &&
         corner.x - 0.1f < precurve_distance + distance_to_corner;
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
