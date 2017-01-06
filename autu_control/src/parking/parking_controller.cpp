#include "autu_control/parking/parking_controller.h"

#include <cmath>

#include "pses_basis/Command.h"

#include "tf/tf.h"

#define DETECT_CORNER 5
#define DRIVE_TO_CORNER 6
#define TURN_RIGHT_INIT 0
#define TURN_RIGHT 1
#define STRAIGHT 2
#define TURN_LEFT 3
#define STOP 4

using Eigen::Vector2f;
using std::abs;

ParkingController::ParkingController(ros::NodeHandle &nh)
    : laserUtil(nh), state(DETECT_CORNER) {
  command_pub = nh.advertise<pses_basis::Command>("autu/command", 1);

  odom_sub = nh.subscribe<nav_msgs::Odometry>(
      "/odom", 1, &ParkingController::odomCallback, this);

  laser_sub = nh.subscribe<sensor_msgs::LaserScan>(
      "/scan", 1, &ParkingController::laserscanCallback, this);

  velocity = nh.param<int>("main/parking/velocity", 5);

  maxSteering = nh.param<int>("main/parking/max_steering", 42);

  maxAngle = nh.param<float>("main/parking/max_angle", M_PI_4);

  regulator_d = nh.param<float>("main/parking/regulator_d", 3);
  regulator_p = nh.param<float>("main/parking/regulator_p", 10);

  a = nh.param<float>("main/parking/a", 0.4f);
  b = nh.param<float>("main/parking/b", 0.32f);
  w = nh.param<float>("main/parking/w", 0.2f);

  pidRegler = PIDRegler(nh, regulator_p, regulator_d, velocity, (a + w) / 2);
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
  if (odom == nullptr || laserscan == nullptr)
    return;
  Vector2f vec;
  switch (state) {
  case DETECT_CORNER:
    vec = laserUtil.findCorner(laserscan);
    corner.position.x = vec[0];
    corner.position.y = vec[1];
    state = DRIVE_TO_CORNER;
    break;
  case DRIVE_TO_CORNER:
    if (odom->pose.pose.position.x > corner.position.x + 0.5f)
      state = TURN_RIGHT_INIT;
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
    pidRegler.drive(odom->pose.pose.position.y - corner.position.y, false);
    break;
  case TURN_RIGHT:
    cmd.motor_level = -velocity;
    cmd.steering_level = -maxSteering;
    command_pub.publish(cmd);
    break;
  case STRAIGHT:
    cmd.motor_level = 0;
    cmd.steering_level = 0;
    command_pub.publish(cmd);
    break;
  case TURN_LEFT:
    cmd.motor_level = -velocity;
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
