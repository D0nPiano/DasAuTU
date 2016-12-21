#include "autu_control/emergency_brake.h"

EmergencyBrake::EmergencyBrake(ros::NodeHandle *n) : maxMotorLevel(13) {
  command_pub = n->advertise<pses_basis::Command>("pses_basis/command", 10);
}

void EmergencyBrake::gridMapCallback(
    const nav_msgs::OccupancyGridConstPtr &msg) {}

void EmergencyBrake::commandCallback(const pses_basis::CommandConstPtr &cmd) {
  // copy message to avoid crazy shit
  pses_basis::Command replacement = *cmd;

  // limit motor level
  if (replacement.motor_level > maxMotorLevel)
    replacement.motor_level = maxMotorLevel;

  // publish new command on output topic
  command_pub.publish(replacement);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "emergency_brake");
  ros::NodeHandle n;
  EmergencyBrake emergencyBrake(&n);
  ROS_INFO("Emergency Brake launched");

  ros::Subscriber command_sub = n.subscribe<pses_basis::Command>(
      "autu/command", 10, std::bind(&EmergencyBrake::commandCallback,
                                    emergencyBrake, std::placeholders::_1));

  ros::spin();
  return 0;
}
