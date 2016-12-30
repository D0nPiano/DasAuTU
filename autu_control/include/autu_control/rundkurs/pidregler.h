#ifndef PIDREGLER_H
#define PIDREGLER_H

#include "ros/ros.h"

#include "autu_control/rundkurs/laserDetector.h"

class PIDRegler {
public:
  PIDRegler(ros::NodeHandle &nh);
  void drive(float ldist, float wallDist);
  bool isReady();

  void setLaserDetector(const LaserDetector &detector);

private:
  float e0;
  double t0;
  const LaserDetector *laserDetector;
  ros::Publisher command_pub;
};

#endif // PIDREGLER_H