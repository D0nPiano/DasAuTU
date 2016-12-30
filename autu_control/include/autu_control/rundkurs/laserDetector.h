#ifndef _LaserDetector_H_
#define _LaserDetector_H_

#include "pses_basis/SensorData.h"
#include "ros/ros.h"

#include <sensor_msgs/LaserScan.h>

class LaserDetector {
public:
  LaserDetector();
  ~LaserDetector();
  void initialize();
  bool isNextToCorner() const;
  bool isNextToWall() const;
  float getAngleToWall() const;
  float getAngleToWallInDeg() const;
  float getDistanceToCorner() const;
  float getDistanceToWall() const;

  void setCurrentLaserScan(const sensor_msgs::LaserScanConstPtr &value);

private:
  float calculateBeta(int angleBegin, float &alpha) const;
  sensor_msgs::LaserScanConstPtr currentLaserScan;
  int RANGE_START;
  int RANGE_DIFF;
  float CORNER_SENSITIVITY;
};

#endif
