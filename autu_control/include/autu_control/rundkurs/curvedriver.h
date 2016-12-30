#ifndef CURVEDRIVER_H
#define CURVEDRIVER_H

#include "ros/ros.h"

class CurveDriver {
public:
  CurveDriver(ros::NodeHandle &nh);
  void reset();
  void drive(float ldist, float angleToWall);
  bool isAroundTheCorner();

private:
  ros::Publisher command_pub;
  double curveBegin;
  float driveStraightTime;
  float driveCurveTime;
  float cornerBeginAngle;
};

#endif // CURVEDRIVER_H
