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
using Eigen::Rotation2Df;
using std::abs;
using std::sqrt;
using std::sin;
using std::acos;
using std::asin;

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

  regulator_d = nh.param<float>("main/parking/regulator_d", 3);
  regulator_p = nh.param<float>("main/parking/regulator_p", 10);

  a = nh.param<float>("main/parking/a", 0.4f);
  b = nh.param<float>("main/parking/b", 0.32f);
  w = nh.param<float>("main/parking/w", 0.2f);

  r = nh.param<float>("main/parking/minimal_radius", 1);

  r_e = sqrt(r * r + b * b);

  alpha = acos(1 - b * b / (4 * r * r));

  beta = asin(sin(alpha) * r / r_e);

  theta = acos((r - w) / r_e) - beta;

  delta = theta - alpha;

  pidRegler = PIDRegler(nh, regulator_p, regulator_d, velocity_forward, 0);

#ifndef NDEBUG
  trajectory_pub =
      nh.advertise<nav_msgs::Path>("autu/debug/parking_trajectory", 1);
#endif
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

    try {
      tf::StampedTransform transform;
      transformListener.waitForTransform("base_laser", "/odom", ros::Time(0),
                                         ros::Duration(0.1));
      transformListener.lookupTransform("base_laser", "/odom", ros::Time(0),
                                        transform);

      geometry_msgs::PointStamped cornerInBaseLaser, startInBaseLaser;

      vec[1] += 0.1f;
      cornerInBaseLaser.point.x = vec[0];
      cornerInBaseLaser.point.y = vec[1];
      cornerInBaseLaser.header.frame_id = "/base_laser";
      cornerInBaseLaser.header.stamp = ros::Time(0);
      transformListener.transformPoint("/odom", cornerInBaseLaser, corner);

      startInBaseLaser.point.x = vec[0] + r * sin(delta);
      startInBaseLaser.point.y = vec[1] + r * (1 - cos(delta));
      startInBaseLaser.header.frame_id = "/base_laser";
      startInBaseLaser.header.stamp = ros::Time(0);
      transformListener.transformPoint("/odom", startInBaseLaser, start);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }
#ifndef NDEBUG
    publishParkingTrajectory();
#endif
    state = DRIVE_TO_CORNER;
    break;
  case DRIVE_TO_CORNER:

    try {
      tf::StampedTransform transform;
      transformListener.waitForTransform("/odom", "/rear_right_wheel",
                                         ros::Time(0), ros::Duration(0.1));
      transformListener.lookupTransform("/odom", "/rear_right_wheel",
                                        ros::Time(0), transform);

      geometry_msgs::PointStamped startInRightWheel;
      start.header.stamp = ros::Time(0);
      transformListener.transformPoint("/rear_right_wheel", start,
                                       startInRightWheel);
      ROS_INFO("odom.y: %f start.y: %f dif: %f", odom->pose.pose.position.y,
               start.point.y, odom->pose.pose.position.y - start.point.y);
      if (startInRightWheel.point.x < 0)
        state = TURN_RIGHT_INIT;
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }
    break;
  case TURN_RIGHT_INIT:
    curveBegin = odom->pose.pose;
    state = TURN_RIGHT;
    break;
  case TURN_RIGHT:
    tf::quaternionMsgToTF(curveBegin.orientation, begin);
    tf::quaternionMsgToTF(odom->pose.pose.orientation, current);
    if (2 * begin.angle(current) > theta)
      state = STRAIGHT;
    break;
  case STRAIGHT:
    curveBegin = odom->pose.pose;
    state = TURN_LEFT;
    break;
  case TURN_LEFT:
    tf::quaternionMsgToTF(curveBegin.orientation, begin);
    tf::quaternionMsgToTF(odom->pose.pose.orientation, current);
    if (2 * begin.angle(current) > theta)
      state = STOP;
    break;
  default:
    break;
  }

  pses_basis::Command cmd;
  switch (state) {
  case DRIVE_TO_CORNER:
    pidRegler.drive(odom->pose.pose.position.y - start.point.y, false);
    break;
  case TURN_RIGHT:
    cmd.motor_level = -velocity_backward;
    cmd.steering_level = -maxSteering;
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

void ParkingController::publishParkingTrajectory() {
  nav_msgs::Path msg;
  geometry_msgs::PoseStamped pose;
  msg.header.frame_id = "odom";

  Vector2f start;
  start[0] = r * sin(delta);
  start[1] = r * (1 - cos(delta));

  Vector2f startICC = start;
  startICC[1] -= r;

  pose.pose.position = corner.point;
  pose.pose.position.x += start[0];
  pose.pose.position.y += start[1];
  msg.poses.push_back(pose);

  const Vector2f v_r(0, r);

  const int intervals = 20;
  for (int i = 0; i <= intervals; ++i) {
    Rotation2Df rot(theta * i / intervals);

    const Vector2f vec = rot * v_r + startICC;

    pose.pose.position = corner.point;
    pose.pose.position.x += vec[0];
    pose.pose.position.y += vec[1];
    msg.poses.push_back(pose);
  }

  const Vector2f endICC = Rotation2Df(theta) * (2 * v_r) + startICC;

  for (int i = intervals; i >= 0; --i) {
    Rotation2Df rot(theta * i / intervals);

    const Vector2f vec = rot * -v_r + endICC;

    pose.pose.position = corner.point;
    pose.pose.position.x += vec[0];
    pose.pose.position.y += vec[1];
    msg.poses.push_back(pose);
  }

  trajectory_pub.publish(msg);
}
