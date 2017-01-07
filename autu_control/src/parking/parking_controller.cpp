#include "autu_control/parking/parking_controller.h"

#include <cmath>

#include "pses_basis/Command.h"

#define DETECT_CORNER 5
#define DRIVE_TO_CORNER 6
#define TURN_RIGHT_INIT 0
#define TURN_RIGHT 1
#define STRAIGHT 2
#define TURN_LEFT 3
#define STOP 4

using Eigen::Vector2f;
using std::abs;
using std::sqrt;

ParkingController::ParkingController(ros::NodeHandle &nh)
    : laserUtil(nh), state(DETECT_CORNER) {
  command_pub = nh.advertise<pses_basis::Command>("autu/command", 1);

  odom_sub = nh.subscribe<nav_msgs::Odometry>(
      "/odom", 1, &ParkingController::odomCallback, this);

  laser_sub = nh.subscribe<sensor_msgs::LaserScan>(
      "/scan", 1, &ParkingController::laserscanCallback, this);

  velocity_forward = nh.param<int>("main/parking/velocity_forward", 5);
  velocity_backward = nh.param<int>("main/parking/velocity_backward", 5);

  maxSteering = nh.param<int>("main/parking/max_steering", 42);

  maxAngle = nh.param<float>("main/parking/max_angle", M_PI_4);

  regulator_d = nh.param<float>("main/parking/regulator_d", 3);
  regulator_p = nh.param<float>("main/parking/regulator_p", 10);

  a = nh.param<float>("main/parking/a", 0.4f);
  b = nh.param<float>("main/parking/b", 0.32f);
  w = nh.param<float>("main/parking/w", 0.2f);

  pidRegler = PIDRegler(nh, regulator_p, regulator_d, velocity_forward, a / 2);
}

void ParkingController::odomCallback(const nav_msgs::OdometryConstPtr &msg) {
  odom = msg;
}

void ParkingController::laserscanCallback(
    const sensor_msgs::LaserScanConstPtr &msg) {
  laserscan = msg;
}

void ParkingController::run() {
  tf::Quaternion begin, current;
  if (odom == nullptr || laserscan == nullptr ||
      !transformListener.canTransform("/rear_right_wheel", "/odom",
                                      ros::Time(0)) ||
      !transformListener.canTransform("/base_laser", "/odom", ros::Time(0)))
    return;
  Vector2f vec;
  switch (state) {
  case DETECT_CORNER:
    vec = laserUtil.findCorner(laserscan);
    state = 42;
    return;

    // corner.position.x = vec[0];
    // corner.position.y = vec[1];
    try {
      tf::StampedTransform transform;
      transformListener.waitForTransform("base_laser", "/odom", ros::Time(0),
                                         ros::Duration(0.1));
      transformListener.lookupTransform("base_laser", "/odom", ros::Time(0),
                                        transform);

      geometry_msgs::PointStamped cornerInBaseLaser;
      cornerInBaseLaser.point.x = vec[0];
      cornerInBaseLaser.point.y = vec[1];
      cornerInBaseLaser.header.frame_id = "/base_laser";
      cornerInBaseLaser.header.stamp = ros::Time(0);
      transformListener.transformPoint("/odom", cornerInBaseLaser, corner);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }
    state = DRIVE_TO_CORNER;
    break;
  case DRIVE_TO_CORNER:

    try {
      tf::StampedTransform transform;
      transformListener.waitForTransform("/rear_right_wheel", "/odom",
                                         ros::Time(0), ros::Duration(0.1));
      transformListener.lookupTransform("/rear_right_wheel", "/odom",
                                        ros::Time(0), transform);

      geometry_msgs::PointStamped pointRightWheel, wheelInOdom;
      pointRightWheel.point.y = -(a - w) / 2;
      pointRightWheel.header.frame_id = "/rear_right_wheel";
      pointRightWheel.header.stamp = ros::Time(0);
      transformListener.transformPoint("/odom", pointRightWheel, wheelInOdom);
      const float xDif = corner.point.x - wheelInOdom.point.x;
      const float yDif = corner.point.y - wheelInOdom.point.y;
      const float distance = sqrt(xDif * xDif + yDif * yDif);
      ROS_INFO("Distance: %f", distance);
      if (distance < 0.1f)
        state = TURN_RIGHT_INIT;
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }
    // if (odom->pose.pose.position.x > corner.position.x + 0.5f)
    // state = TURN_RIGHT_INIT;
    break;
  case TURN_RIGHT_INIT:
    curveBegin = odom->pose.pose;
    state = TURN_RIGHT;
    break;
  case TURN_RIGHT:
    tf::quaternionMsgToTF(curveBegin.orientation, begin);
    tf::quaternionMsgToTF(odom->pose.pose.orientation, current);
    if (2 * begin.angle(current) > maxAngle)
      state = STRAIGHT;
    break;
  case STRAIGHT:
    curveBegin = odom->pose.pose;
    state = TURN_LEFT;
    break;
  case TURN_LEFT:
    tf::quaternionMsgToTF(curveBegin.orientation, begin);
    tf::quaternionMsgToTF(odom->pose.pose.orientation, current);
    if (2 * begin.angle(current) > maxAngle)
      state = STOP;
    break;
  default:
    break;
  }

  pses_basis::Command cmd;
  switch (state) {
  case DRIVE_TO_CORNER:
    pidRegler.drive(odom->pose.pose.position.y - corner.point.y, false);
    break;
  case TURN_RIGHT:
    cmd.motor_level = -velocity_backward;
    cmd.steering_level = -maxSteering;
    command_pub.publish(cmd);
    break;
  case STRAIGHT:
    cmd.motor_level = 0;
    cmd.steering_level = 0;
    command_pub.publish(cmd);
    break;
  case TURN_LEFT:
    cmd.motor_level = -velocity_backward;
    cmd.steering_level = maxSteering;
    command_pub.publish(cmd);
    break;
  case STOP:
    cmd.motor_level = 0;
    cmd.steering_level = maxSteering;
    command_pub.publish(cmd);
    break;
  default:
    break;
  }
}
