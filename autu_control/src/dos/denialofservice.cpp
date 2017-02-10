#include "autu_control/dos/denialofservice.h"

DenialOfService::DenialOfService(ros::NodeHandle &n) {
  cmd_pub = n.advertise<pses_basis::Command>("pses_basis/command", 1);

  timer = n.createTimer(ros::Duration(0.001),
                        &DenialOfService::sendMotorCommand, this);
}

void DenialOfService::run() {}

void DenialOfService::sendMotorCommand(const ros::TimerEvent &) {
  pses_basis::Command cmd;
  cmd.motor_level = 5;
  ++i;
  cmd.steering_level = i % 2 ? 50 : -50;
  if (i > 4000)
    i = 0;
  cmd_pub.publish(cmd);
}
