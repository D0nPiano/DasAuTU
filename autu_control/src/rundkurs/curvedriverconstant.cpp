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
using Eigen::Vector2f;

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

  // angle() returns only the current angle divided by 2
  return start.angle(current) > cornerEndAngle * M_PI_2 / 180.0f;
}

void CurveDriverConstant::curveInit() {
  // remember the car's position and orientation at the beginning of the curve
  curveBegin = odom->pose.pose;
}

bool CurveDriverConstant::isNextToGlas(float cornerX, float cornerY) {

  // distance to the end of the curve area
  const float maxDist = cornerX + 1.2f;

  // last angle which is relevant for the curve area
  const float minAlpha = M_PI_2 - atan(maxDist / cornerY);

  // counter for points in the curve area
  int counter = 0;

  // iterate from left to the center of the scan
  for (size_t i = laserscan->ranges.size() - 1;
       i > laserscan->ranges.size() / 2; --i) {

    const float r = laserscan->ranges[i];

    // ignore invalid scan values
    if (laserscan->range_min < r && r < laserscan->range_max) {
      // angle of the current point
      const float alpha = laserscan->angle_min + i * laserscan->angle_increment;

      // points are not more in the curve area
      if (alpha < minAlpha)
        break;

      // convert to cartesian coordinates
      const float x = r * cos(alpha);
      const float y = r * sin(alpha);

      if (cornerX + 0.1f < x && x < maxDist)
        if (y < cornerY + 0.5f)
          // count points which are in the curve area
          ++counter;
    }
  }

  // if too many points are in the area of the curve, it is not a real curve
  return counter >= 5;
}

bool CurveDriverConstant::wallFound(float cornerX, float cornerY) {
  // angle of the detected corner in the scan
  const float minAlpha = M_PI_2 - atan(cornerX / cornerY);

  // counter for points ahead of the corner
  int counter = 0;

  // iterate from left to the center of the scan
  for (size_t i = laserscan->ranges.size() - 1;
       i > laserscan->ranges.size() / 2; --i) {

    const float r = laserscan->ranges[i];

    // ignore invalid scan values
    if (laserscan->range_min < r && r < laserscan->range_max) {
      // angle of the current point
      const float alpha = laserscan->angle_min + i * laserscan->angle_increment;

      // points are now after the corner and must be ignored
      if (alpha < minAlpha)
        break;

      // convert to cartesian coordinates
      const float x = r * cos(alpha);
      const float y = r * sin(alpha);

      if (x < cornerX)
        if (cornerY - 0.3f < y && y < cornerY + 0.3f)
          ++counter;
    }
  }

  // at least 80 point must be near the corner or it was a false-positive
  return counter >= 80;
}

bool CurveDriverConstant::isNextToCorner(float speed) {
  if (laserscan == nullptr)
    return false;

  // calculate laserscan delay
  updateScanOffset(speed);

  // vector pointing to the detected corner
  Vector2f vecToCorner;
  float last_r = std::numeric_limits<float>::max();

  size_t i;
  // iterate from left to the center of the scan
  for (i = laserscan->ranges.size() - 1; i > laserscan->ranges.size() / 2;
       --i) {

    const float r = laserscan->ranges[i];

    // ignore invalid points and points which are too close to be valid
    if (laserscan->range_min < r && 0.4f < r && r < laserscan->range_max) {
      if (r - last_r > cornerThreshold)
        // big step between 2 values detected -> curve possibly found
        break;
      else
        // remember last point
        last_r = r;
    }
  }

  // angle of the detected corner
  const float alpha =
      abs(laserscan->angle_min + (i + 1) * laserscan->angle_increment);

  // consider laserscan delay
  corner.x = last_r * cos(alpha) - scanOffset;
  corner.y = last_r * sin(alpha);

  // corner is too far away to be analyzed correctly
  if (corner.x > 4.0f)
    return false;

  // convert corner to cartesian coordinates
  vecToCorner[0] = last_r * cos(alpha);
  vecToCorner[1] = last_r * sin(alpha);

  // remember where the car was when the curve was detected
  cornerSeen = odom->pose.pose;

  /*const float vc = maxMotorLevel / 10.0f;
  const float dif = vc - speed;
   rollout_distance = dif * dif / -2 + speed * (speed - vc);

   if (rollout_distance < 0)
     rollout_distance = 0;*/

  // rollout is ignored
  rolloutDistance = 0;

  std_msgs::String debugMsg;
  if (!isNextToGlas(vecToCorner[0], vecToCorner[1])) {
    if (wallFound(vecToCorner[0], vecToCorner[1])) {
      if (corner.x - 0.1f <
          precurveDistance + rolloutDistance + blindnessOffset) {

        // publish the detected corner
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
    // a new scan was received so reset the scanOffset
    scanOffset = 0;
    scanOffsetStamp = scan->header.stamp.toSec();
    laserscan = scan;
  }
}

float CurveDriverConstant::getCurrentDistanceToCorner() const {
  const float xDif = cornerInOdom.point.x - odom->pose.pose.position.x;
  const float yDif = cornerInOdom.point.y - odom->pose.pose.position.y;

  // calculate euclidean distance
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
