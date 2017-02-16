#include "autu_control/rundkurs/pidregler.h"

#include <math.h>

#include "pses_basis/Command.h"

PIDRegler::PIDRegler(ros::NodeHandle &nh) : e0(0), t0(0) {
  command_pub = nh.advertise<pses_basis::Command>("autu/command", 1);

  maxMotorLevel = nh.param<int>("main/pidregler/max_motor_level", 8);
  p = nh.param<float>("main/pidregler/p", 16.0f);
  d = nh.param<float>("main/pidregler/d", 8.0f);
  solldist = nh.param<float>("main/pidregler/solldist", 0.9f);
  obstacleDist = 0.0;
  avoidDirection = true;
}

PIDRegler::PIDRegler(ros::NodeHandle &nh, float p, float d, int maxMotorLevel,
                     float solldist)
    : e0(0), t0(0), maxMotorLevel(maxMotorLevel), p(p), d(d),
      solldist(solldist) {
  command_pub = nh.advertise<pses_basis::Command>("autu/command", 1);
  obstacleDist = 0.0;
  avoidDirection = true;
}

void PIDRegler::reset() {
  e0 = 0;
  t0 = 0;
  obstacleDist = 0.0;
}

void PIDRegler::drive(float ldist, bool left) {
  pses_basis::Command cmd;

  // float parallelAngle =  -(90 - laserDetector->getAngleToWallInDeg());
  float wallDist = laserDetector->getDistanceToWall();

  ROS_INFO("Abstand: [%f]", wallDist);
  ROS_INFO("AbstandUS: [%f]", ldist);
  //ldist = ldist + wallDist / 2.0;

  /* TODO: Get corner, set
                  drivingCurve = true;
                  curveBegin = ros::Time::now().toSec();
                  */

  float steerfact = left ? -2.0f : 2.0f;

  obstacleDist *= 0.999;
  ROS_INFO("obstacleDist [%f]", obstacleDist);

  // P-Regler, tb = 62s

  cmd.motor_level = maxMotorLevel;
  // TODO k1 & k2 are unused
  // float k1 = 16;
  // float k2 = 5;
  double t = ros::Time::now().toSec();
  float e = solldist - ldist + obstacleDist;
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

void PIDRegler::avoidObstacle(float ldist) {
  float diff = fabs(solldist - ldist + obstacleDist);
  if(diff > 1){
    diff = 0;
  } else {
    diff = 1 - diff;
  }
  ROS_INFO("Difference [%f]", diff);
  if(avoidDirection){
    obstacleDist += 0.0002 * diff;
    if(obstacleDist > 0.32){ //weiter links
      avoidDirection = false;
    }
  } else {
    obstacleDist -= 0.0002 * diff;
    if(obstacleDist < -0.32){ // weiter rechts
      avoidDirection = true;
    }
  }
}


bool PIDRegler::isReady(float dist) {
  //return laserDetector->isNextToWall() &&
  //     (laserDetector->getAngleToWall() * 180 / M_PI) < 100.0;
  return 0.3f < dist && dist < 1.4f;
}

void PIDRegler::setLaserDetector(const LaserDetector &detector) {
  laserDetector = &detector;
}

void PIDRegler::setMaxMotorLevel(const int16_t &value) {
  maxMotorLevel = value;
}

int16_t PIDRegler::getMaxMotorLevel() const { return maxMotorLevel; }
