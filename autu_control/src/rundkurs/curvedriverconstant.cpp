#include "autu_control/rundkurs/curvedriverconstant.h"

#include "Eigen/Dense"
#include <cmath>
#include <limits>
#include <vector>

#include "pses_basis/Command.h"

using std::sqrt;
using std::atan;
using std::sin;
using std::cos;
using std::abs;

CurveDriverConstant::CurveDriverConstant(ros::NodeHandle &nh) : scanOffset(0) {
  commandPub = nh.advertise<pses_basis::Command>("autu/command", 1);

  cornerPub =
      nh.advertise<geometry_msgs::PoseStamped>("autu/rundkurs/corner", 1);

  infoPub = nh.advertise<std_msgs::String>("autu/rundkurs/info", 10);

  maxMotorLevel = nh.param<int>("main/curvedriver_constant/max_motor_level", 8);
  steering = nh.param<int>("main/curvedriver_constant/steering", 25);

  cornerThreshold =
      nh.param<float>("main/curvedriver_constant/corner_threshold", 0.5f);
  cornerEndAngle =
      nh.param<float>("main/curvedriver_constant/corner_end_angle", 80.0f);
  precurveDistance =
      nh.param<float>("main/curvedriver_constant/precurve_distance", 0.8f);
  blindnessOffset =
      nh.param<float>("main/curvedriver_constant/blindness_offset", 0);
}

void CurveDriverConstant::drive() {
  pses_basis::Command cmd;

  cmd.motor_level = maxMotorLevel;
  cmd.steering_level = steering;

  cmd.header.stamp = ros::Time::now();
  commandPub.publish(cmd);
}

bool CurveDriverConstant::isAroundTheCorner() const {
  tf::Quaternion start, current;
  tf::quaternionMsgToTF(curveBegin.orientation, start);
  tf::quaternionMsgToTF(odom->pose.pose.orientation, current);

  return start.angle(current) > cornerEndAngle * M_PI_2 / 180.0f;
}

void CurveDriverConstant::curveInit() { curveBegin = odom->pose.pose; }

bool CurveDriverConstant::isNextToGlas(float cornerX, float cornerY) {
  const float maxDist = cornerX + 1.2f;
  const float minAlpha = M_PI_2 - atan(maxDist / cornerY);
  int counter = 0;
  for (size_t i = laserscan->ranges.size() - 1;
       i > laserscan->ranges.size() / 2; --i) {
    const float r = laserscan->ranges[i];
    if (laserscan->range_min < r && r < laserscan->range_max) {
      const float alpha = laserscan->angle_min + i * laserscan->angle_increment;

      if (alpha < minAlpha)
        break;

      const float x = r * cos(alpha);
      const float y = r * sin(alpha);

      if (cornerX + 0.1f < x && x < maxDist)
        if (y < cornerY + 0.5f)
          ++counter;
    }
  }
  return counter >= 5;
}

bool CurveDriverConstant::wallFound(float cornerX, float cornerY) {
  const float minAlpha = M_PI_2 - atan(cornerX / cornerY);
  int counter = 0;
  for (size_t i = laserscan->ranges.size() - 1;
       i > laserscan->ranges.size() / 2; --i) {
    const float r = laserscan->ranges[i];
    if (laserscan->range_min < r && r < laserscan->range_max) {
      const float alpha = laserscan->angle_min + i * laserscan->angle_increment;

      if (alpha < minAlpha)
        break;

      const float x = r * cos(alpha);
      const float y = r * sin(alpha);

      if (x < cornerX)
        if (cornerY - 0.3f < y && y < cornerY + 0.3f)
          ++counter;
    }
  }

  return counter >= 80;
}

bool CurveDriverConstant::isNextToCorner(float speed) {
  if (laserscan == nullptr)
    return false;
  updateScanOffset(speed);

  Eigen::Vector2f vecToCorner;
  float last_r = std::numeric_limits<float>::max();

  size_t i;
  for (i = laserscan->ranges.size() - 1; i > laserscan->ranges.size() / 2;
       --i) {
    const float r = laserscan->ranges[i];
    if (laserscan->range_min < r && 0.4f < r && r < laserscan->range_max) {
      if (r - last_r > cornerThreshold)
        break;
      else
        last_r = r;
    }
  }

  const float alpha =
      abs(laserscan->angle_min + (i + 1) * laserscan->angle_increment);

  corner.x = last_r * cos(alpha) - scanOffset;
  corner.y = last_r * sin(alpha);

  if (corner.x > 4.0f)
    return false;

  vecToCorner[0] = last_r * cos(alpha);
  vecToCorner[1] = last_r * sin(alpha);

  cornerSeen = odom->pose.pose;

  /*const float vc = maxMotorLevel / 10.0f;
  const float dif = vc - speed;
   rollout_distance = dif * dif / -2 + speed * (speed - vc);

   if (rollout_distance < 0)
     rollout_distance = 0;*/

  rolloutDistance = 0;

  std_msgs::String debugMsg;
  if (!isNextToGlas(vecToCorner[0], vecToCorner[1])) {
    if (wallFound(vecToCorner[0], vecToCorner[1])) {
      if (corner.x - 0.1f <
          precurveDistance + rolloutDistance + blindnessOffset) {

        try {
          tf::StampedTransform transform;
          transformListener.waitForTransform("/base_laser", "/odom",
                                             laserscan->header.stamp,
                                             ros::Duration(0.1));
          transformListener.lookupTransform("/base_laser", "/odom",
                                            laserscan->header.stamp, transform);

          geometry_msgs::PointStamped cornerInBaseLaser;
          cornerInBaseLaser.header.stamp = laserscan->header.stamp;
          cornerInBaseLaser.header.frame_id = "/base_laser";
          cornerInBaseLaser.point.x = vecToCorner[0];
          cornerInBaseLaser.point.y = vecToCorner[1];
          transformListener.transformPoint("/odom", cornerInBaseLaser,
                                           cornerInOdom);

          geometry_msgs::PoseStamped poseCorner;
          poseCorner.header.frame_id = "/odom";
          poseCorner.header.stamp = laserscan->header.stamp;
          poseCorner.pose.position.x = cornerInOdom.point.x;
          poseCorner.pose.position.y = cornerInOdom.point.y;
          cornerPub.publish(poseCorner);
        } catch (tf::TransformException ex) {
          ROS_ERROR("%s", ex.what());
          return false;
        }

        return true;
      } else
        debugMsg.data = "Curve-condition not fulfilled";
    } else
      debugMsg.data = "No wall found";
  } else
    debugMsg.data = "Next To Glas";
  infoPub.publish(debugMsg);
  return false;
}

bool CurveDriverConstant::rolloutBegins() const {
  return getCurrentDistanceToCorner() < precurveDistance + rolloutDistance;
}

bool CurveDriverConstant::isAtCurveBegin() const {
  return getCurrentDistanceToCorner() < precurveDistance;
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

float CurveDriverConstant::getCurrentDistanceToCorner() const {
  const float xDif = cornerInOdom.point.x - odom->pose.pose.position.x;
  const float yDif = cornerInOdom.point.y - odom->pose.pose.position.y;

  return sqrt(xDif * xDif + yDif * yDif);
}

void CurveDriverConstant::setOdom(const nav_msgs::OdometryConstPtr &msg) {
  odom = msg;
}

void CurveDriverConstant::updateScanOffset(float speed) {
  const double now = ros::Time::now().toSec();
  // time since last update
  const double timeDif = now - scanOffsetStamp;
  scanOffset += timeDif * speed;
  // last update happened now
  scanOffsetStamp = now;
}
