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

CurveDriverConstant::CurveDriverConstant(ros::NodeHandle &nh,
                                         LaserUtil &laserUtil)
    : scanOffset(0), falseCornerDetected(false), laserUtil(laserUtil) {
  command_pub = nh.advertise<pses_basis::Command>("autu/command", 1);

  corner_pub =
      nh.advertise<geometry_msgs::PoseStamped>("autu/rundkurs/corner", 1);

  info_pub = nh.advertise<std_msgs::String>("autu/rundkurs/info", 10);

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
  blindness_offset =
      nh.param<float>("main/curvedriver_constant/blindness_offset", 0);
  radius = nh.param<float>("main/curvedriver_constant/radius", 1.5f);
  cornerSafetyDistance =
      nh.param<float>("main/curvedriver_constant/corner_safety_distance", 0.2f);
  motorLevelFactor =
      nh.param<float>("main/curvedriver_constant/motorlevel_factor", 10.0f);
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

  return start.angle(current) > corner_end_angle * M_PI_2 / 180.0f;
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
  ROS_INFO("counter: %d", counter);
  return counter >= 80;
}

bool CurveDriverConstant::isNextToCorner(float distanceToWall, float speed) {
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
      if (r - last_r > corner_threshold)
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

  /*const float vc = maxMotorLevel / motorLevelFactor;
  const float dif = vc - speed;
   rollout_distance = dif * dif / -2 + speed * (speed - vc);

   if (rollout_distance < 0)
     rollout_distance = 0;*/

  rollout_distance = 0;

  /* precurve_distance =
       sqrt(cornerSafetyDistance * cornerSafetyDistance -
            distanceToWall * distanceToWall +
            2 * radius * (distanceToWall - cornerSafetyDistance));*/
  std_msgs::String msg;
  if (!isNextToGlas(vecToCorner[0], vecToCorner[1])) {
    if (wallFound(vecToCorner[0], vecToCorner[1])) {
      if (corner.x - 0.1f <
          precurve_distance + rollout_distance + blindness_offset) {

        /* geometry_msgs::PoseStamped poseCorner;
         poseCorner.header.frame_id = "base_laser";
         poseCorner.header.stamp = laserscan->header.stamp;
         poseCorner.pose.position.x = vecToCorner[0];
         poseCorner.pose.position.y = vecToCorner[1];*/
        // corner_pub.publish(poseCorner);

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
          corner_pub.publish(poseCorner);
        } catch (tf::TransformException ex) {
          ROS_ERROR("%s", ex.what());
          return false;
        }

        return true;
      } else
        msg.data = "Curve-condition not fulfilled";
    } else
      msg.data = "No wall found";
  } else
    msg.data = "Next To Glas";
  info_pub.publish(msg);
  return false;
}

bool CurveDriverConstant::rolloutBegins() const {
  /* const float xDif = cornerSeen.position.x - odom->pose.pose.position.x;
   const float yDif = cornerSeen.position.y - odom->pose.pose.position.y;

   // distance to the point where the corner was detected
   const float distance = sqrt(xDif * xDif + yDif * yDif);

   return distance > blindness_offset;*/
  const float xDif = cornerInOdom.point.x - odom->pose.pose.position.x;
  const float yDif = cornerInOdom.point.y - odom->pose.pose.position.y;

  // distance to the point where the corner was detected
  const float distance = sqrt(xDif * xDif + yDif * yDif);
  return distance < precurve_distance + rollout_distance;
}

bool CurveDriverConstant::isAtCurveBegin() const {
  /*  const float xDif = cornerSeen.position.x - odom->pose.pose.position.x;
    const float yDif = cornerSeen.position.y - odom->pose.pose.position.y;

    // distance to the point where the corner was detected
    const float distance = sqrt(xDif * xDif + yDif * yDif);

    return distance > blindness_offset + rollout_distance;*/
  const float xDif = cornerInOdom.point.x - odom->pose.pose.position.x;
  const float yDif = cornerInOdom.point.y - odom->pose.pose.position.y;

  // distance to the point where the corner was detected
  const float distance = sqrt(xDif * xDif + yDif * yDif);
  return distance < precurve_distance;
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
