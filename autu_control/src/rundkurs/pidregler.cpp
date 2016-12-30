#include "autu_control/rundkurs/pidregler.h"

#include <math.h>

#include "pses_basis/Command.h"

PIDRegler::PIDRegler(ros::NodeHandle &nh) : e0(0), t0(0) {
  command_pub = nh.advertise<pses_basis::Command>("autu/command", 1);
}

void PIDRegler::drive(float ldist, float wallDist) {
  pses_basis::Command cmd;

  // float parallelAngle =  -(90 - laserDetector->getAngleToWallInDeg());

  ldist = ldist + wallDist / 2.0;

  /* TODO: Get corner, set
                  drivingCurve = true;
                  curveBegin = ros::Time::now().toSec();
                  */

  float solldist = 0.9;
  float steerfact = -2;

  // P-Regler, tb = 62s

  cmd.motor_level = 8;
  float p = 16;
  float d = 8;
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

bool PIDRegler::isReady() {
  return laserDetector->isNextToWall() &&
         (laserDetector->getAngleToWall() * 180 / M_PI) < 100.0;
}

void PIDRegler::setLaserDetector(const LaserDetector &detector) {
  laserDetector = &detector;
}