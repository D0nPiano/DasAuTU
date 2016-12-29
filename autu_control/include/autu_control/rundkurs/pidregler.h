#ifndef PIDREGLER_H
#define PIDREGLER_H

#include "ros/ros.h"

class PIDRegler {
public:
  PIDRegler(ros::NodeHandle &nh);
  void drive(float ldist, float wallDist);

private:
  float e0;
  double t0;
  ros::Publisher command_pub;
};

#endif // PIDREGLER_H
