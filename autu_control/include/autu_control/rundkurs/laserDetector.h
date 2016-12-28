#ifndef _LaserDetector_H_
#define _LaserDetector_H_

#include "pses_basis/SensorData.h"
#include "ros/ros.h"

#include <sensor_msgs/LaserScan.h>

class LaserDetector {
public:
  LaserDetector(const sensor_msgs::LaserScanPtr &);
  ~LaserDetector();
  void initialize();
  bool isNextToCorner();
  bool isNextToWall();
  float getAngleToWall();
  float getAngleToWallInDeg();
  float getDistanceToCorner();
  float getDistanceToWall();

private:
  float calculateBeta(int angleBegin, float &alpha);
  sensor_msgs::LaserScanConstPtr currentLaserScan;
  int RANGE_START;
  int RANGE_DIFF;
  float CORNER_SENSITIVITY;
};

#endif
