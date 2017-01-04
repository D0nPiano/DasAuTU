#include "autu_control/parking/parking_controller.h"

#include <cmath>

#include "pses_basis/Command.h"

#include "tf/tf.h"

#define TURN_RIGHT_INIT 0
#define TURN_RIGHT 1
#define STRAIGHT 2
#define TURN_LEFT 3
#define STOP 4

ParkingController::ParkingController(ros::NodeHandle &nh)
    : state(TURN_RIGHT_INIT) {
  command_pub = nh.advertise<pses_basis::Command>("autu/command", 1);

  odom_sub = nh.subscribe<nav_msgs::Odometry>(
      "/odom", 1, &ParkingController::odomCallback, this);

  velocity = nh.param<int>("main/parking/velocity", 5);

  maxSteering = nh.param<int>("main/parking/max_steering", 42);
}

void ParkingController::odomCallback(const nav_msgs::OdometryConstPtr &msg) {
  odom = msg;
}

void ParkingController::run() {
  tf::Quaternion begin, current;
  if (odom == nullptr)
    return;
  switch (state) {
  case TURN_RIGHT_INIT:
    curveBegin = odom->pose.pose;
    state = TURN_RIGHT;
    break;
  case TURN_RIGHT:
    tf::quaternionMsgToTF(curveBegin.orientation, begin);
    tf::quaternionMsgToTF(odom->pose.pose.orientation, current);
    if (2 * begin.angle(current) > M_PI_4)
      state = STRAIGHT;
    break;
  case STRAIGHT:
    curveBegin = odom->pose.pose;
    state = TURN_LEFT;
    break;
  case TURN_LEFT:
    tf::quaternionMsgToTF(curveBegin.orientation, begin);
    tf::quaternionMsgToTF(odom->pose.pose.orientation, current);
    if (2 * begin.angle(current) > M_PI_4)
      state = STOP;
    break;
  default:
    break;
  }

  pses_basis::Command cmd;
  switch (state) {
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
