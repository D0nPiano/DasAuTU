#ifndef PIDREGLER_H
#define PIDREGLER_H

#include "ros/ros.h"

#include "autu_control/rundkurs/laserDetector.h"

class PIDRegler {
public:
  PIDRegler(ros::NodeHandle &nh);
  void reset();
  void drive(float ldist);
  bool isReady();

  void setLaserDetector(const LaserDetector &detector);

private:
  float e0;
  double t0;
  int16_t maxMotorLevel;
  const LaserDetector *laserDetector;
  ros::Publisher command_pub;
};

#endif // PIDREGLER_H
