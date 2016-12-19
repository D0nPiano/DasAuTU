#include "autu_control/gamepad/ps3_controller.h"
#include <functional>
#include <pses_basis/Command.h>

#define PS3_AXIS_BUTTON_REAR_RIGHT_2 13
#define PS3_AXIS_STICK_LEFT_LEFTWARDS 0

PS3_Controller::PS3_Controller(ros::NodeHandle *n, ros::Publisher *command_pub)
    : n(n), command_pub(command_pub), r2(0), joystickLeftLeftwards(0) {
  ROS_INFO("PS3 Controller launched");
  joystick_pub = n->subscribe<sensor_msgs::Joy>(
      "joy", 10,
      std::bind(&PS3_Controller::handleJoystick, this, std::placeholders::_1));
}

void PS3_Controller::handleJoystick(const sensor_msgs::Joy::ConstPtr &msg) {

  // ROS_INFO("gas: %f", -msg->axes[13]);
  // r2 range is from 0.0 (not pressed) to 1.0 (full pressed)
  r2 = -msg->axes[PS3_AXIS_BUTTON_REAR_RIGHT_2];
  // joystickLeftLeftwards range is from -1.0 (right) to 1.0 (left)
  joystickLeftLeftwards = msg->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS];
}
void PS3_Controller::run() {
  pses_basis::Command cmd;
  cmd.motor_level = 10 * r2;
  cmd.steering_level = 35 * joystickLeftLeftwards;
  cmd.header.stamp = ros::Time::now();
  command_pub->publish(cmd);
  ros::spinOnce();
}

PS3_Controller::~PS3_Controller() {
  ROS_INFO("Destroying PS3 Controller");
  joystick_pub.shutdown();
}
