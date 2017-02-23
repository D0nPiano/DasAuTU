#include "autu_control/rundkurs/pidregler.h"

#include <math.h>

#include "pses_basis/Command.h"

PIDRegler::PIDRegler(ros::NodeHandle &nh) : e0(0), t0(0) {
  command_pub = nh.advertise<pses_basis::Command>("autu/command", 1);

  maxMotorLevel = nh.param<int>("main/pidregler/max_motor_level", 8);
  p = nh.param<float>("main/pidregler/p", 16.0f);
  d = nh.param<float>("main/pidregler/d", 8.0f);
  solldist = nh.param<float>("main/pidregler/solldist", 0.9f);
}

PIDRegler::PIDRegler(ros::NodeHandle &nh, float p, float d, int maxMotorLevel,
                     float solldist)
    : e0(0), t0(0), maxMotorLevel(maxMotorLevel), p(p), d(d),
      solldist(solldist) {
  command_pub = nh.advertise<pses_basis::Command>("autu/command", 1);
}

void PIDRegler::reset() {
  e0 = 0;
  t0 = 0;
}

void PIDRegler::drive(float ldist, bool left) {
  pses_basis::Command cmd;

  // float parallelAngle =  -(90 - laserDetector->getAngleToWallInDeg());

  // ldist = ldist + wallDist / 2.0;

  /* TODO: Get corner, set
                  drivingCurve = true;
                  curveBegin = ros::Time::now().toSec();
                  */

  float steerfact = left ? -2.0f : 2.0f;

  // P-Regler, tb = 62s

  cmd.motor_level = maxMotorLevel;
  // TODO k1 & k2 are unused
  // float k1 = 16;
  // float k2 = 5;
  double t = ros::Time::now().toSec();
  float e = solldist - ldist;
  cmd.steering_level = steerfact * p * (e + (e - e0) * d / (t - t0));
  // ROS_INFO("parallelAngle:[%f]", parallelAngle);
  // cmd.steering_level = (solldist*16 - (k1 * ldist + k2 * parallelAngle )) *
  // steerfact;

  e0 = e;
  t0 = t;

  if (cmd.steering_level > 40)
    cmd.steering_level = 40;
  else if (cmd.steering_level < -40)
    cmd.steering_level = -40;

  cmd.header.stamp = ros::Time::now();
  command_pub.publish(cmd);
  ros::spinOnce();
}

void PIDRegler::setMaxMotorLevel(const int16_t &value) {
  maxMotorLevel = value;
}

int16_t PIDRegler::getMaxMotorLevel() const { return maxMotorLevel; }
