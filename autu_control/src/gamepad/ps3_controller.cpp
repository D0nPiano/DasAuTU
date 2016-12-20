#include "autu_control/gamepad/ps3_controller.h"
#include <functional>
#include <pses_basis/Command.h>

#define PS3_AXIS_BUTTON_REAR_LEFT_2 12
#define PS3_AXIS_BUTTON_REAR_RIGHT_2 13
#define PS3_AXIS_STICK_LEFT_LEFTWARDS 0

PS3_Controller::PS3_Controller(ros::NodeHandle *n, ros::Publisher *command_pub)
    : n(n), command_pub(command_pub), gaspedal(0), joystickLeftLeftwards(0),
      max_motorlevel(10), max_steering(35) {
  ROS_INFO("PS3 Controller launched");

  // Loading parameters
  if (!n->getParam("/joy_node/max_motorlevel", max_motorlevel))
    ROS_INFO("max_motorlevel not set -> using default value");
  if (!n->getParam("/joy_node/max_steering", max_steering))
    ROS_INFO("max_steering not set -> using default value");

  joystick_sub = n->subscribe<sensor_msgs::Joy>(
      "joy", 10,
      std::bind(&PS3_Controller::handleJoystick, this, std::placeholders::_1));
}

void PS3_Controller::handleJoystick(const sensor_msgs::Joy::ConstPtr &msg) {
  // the right button has a higher priority than the left
  if (msg->axes[PS3_AXIS_BUTTON_REAR_RIGHT_2] != 0)
    // PS3_AXIS_BUTTON_REAR_RIGHT_2 is full pressed -> msg->axes[..] = -1,0
    gaspedal = -msg->axes[PS3_AXIS_BUTTON_REAR_RIGHT_2];
  else
    // PS3_AXIS_BUTTON_REAR_LEFT_2 is full pressed -> msg->axes[..] = -1,0
    gaspedal = msg->axes[PS3_AXIS_BUTTON_REAR_LEFT_2];

  // joystickLeftLeftwards range is from -1.0 (right) to 1.0 (left)
  joystickLeftLeftwards = msg->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS];
}
void PS3_Controller::run() {
  pses_basis::Command cmd;

  cmd.motor_level = max_motorlevel * gaspedal;
  cmd.steering_level = max_steering * joystickLeftLeftwards;

  cmd.header.stamp = ros::Time::now();
  command_pub->publish(cmd);
  ros::spinOnce();
}

PS3_Controller::~PS3_Controller() {
  ROS_INFO("Destroying PS3 Controller");
  joystick_sub.shutdown();
}
