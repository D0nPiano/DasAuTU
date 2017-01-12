#ifndef PIDREGLER_H
#define PIDREGLER_H

#include "ros/ros.h"

#include "autu_control/rundkurs/laserDetector.h"

class PIDRegler {
public:
  PIDRegler() {}
  PIDRegler(ros::NodeHandle &nh);
  PIDRegler(ros::NodeHandle &nh, float p, float d, int maxMotorLevel,
            float solldist);
  void reset();
  void drive(float ldist, bool left);
  bool isReady();

  void setLaserDetector(const LaserDetector &detector);

private:
  float e0;
  double t0;
  int16_t maxMotorLevel;
  float p;
  float d;
  float solldist;
  const LaserDetector *laserDetector;
  ros::Publisher command_pub;
};

#endif // PIDREGLER_H
