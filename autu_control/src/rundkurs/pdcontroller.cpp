#include "autu_control/rundkurs/pdcontroller.h"

#include <math.h>

#include "pses_basis/Command.h"

PDController::PDController(ros::NodeHandle &nh) : e0(0), t0(0) {
  command_pub = nh.advertise<pses_basis::Command>("autu/command", 1);

  maxMotorLevel = nh.param<int>("main/PDController/max_motor_level", 8);
  p = nh.param<float>("main/PDController/p", 16.0f);
  d = nh.param<float>("main/PDController/d", 8.0f);
  solldist = nh.param<float>("main/PDController/solldist", 0.9f);
}

PDController::PDController(ros::NodeHandle &nh, float p, float d, int maxMotorLevel,
                     float solldist)
    : e0(0), t0(0), maxMotorLevel(maxMotorLevel), p(p), d(d),
      solldist(solldist) {
  command_pub = nh.advertise<pses_basis::Command>("autu/command", 1);
}

void PDController::reset() {
  e0 = 0;
  t0 = 0;
}

void PDController::drive(float ldist, bool left) {
  pses_basis::Command cmd;


  float steerfact = left ? -2.0f : 2.0f;


  cmd.motor_level = maxMotorLevel;

  double t = ros::Time::now().toSec();
  float e = solldist - ldist;
  cmd.steering_level = steerfact * p * (e + (e - e0) * d / (t - t0));


  e0 = e;
  t0 = t;
  //max sttering level
  if (cmd.steering_level > 40)
    cmd.steering_level = 40;
  else if (cmd.steering_level < -40)
    cmd.steering_level = -40;

  cmd.header.stamp = ros::Time::now();
  command_pub.publish(cmd);
  ros::spinOnce();
}

void PDController::setMaxMotorLevel(const int16_t &value) {
  maxMotorLevel = value;
}

int16_t PDController::getMaxMotorLevel() const { return maxMotorLevel; }
