#ifndef _LaserDetector_CPP_
#define _LaserDetector_CPP_

#include "autu_control/rundkurs/laserDetector.h"

#define PI 3.14159265
#define ANGLE_OFFSET 0.0

LaserDetector::LaserDetector() {
  ROS_INFO("New LaserDetector");
  CORNER_SENSITIVITY = 2.2;
  RANGE_START = 349;
  RANGE_DIFF = 0;
}

LaserDetector::~LaserDetector() { ROS_INFO("Destroying LaserDetector"); }

void LaserDetector::initialize() {
  float viewangle = currentLaserScan->angle_max - currentLaserScan->angle_min;
  int steps = (int)(viewangle / currentLaserScan->angle_increment);

  RANGE_DIFF = (int)steps / 30.0;
  RANGE_START = steps - 2 * RANGE_DIFF;

  ROS_INFO("RANGE_START: [%d]", RANGE_START);
  ROS_INFO("RANGE_DIFF: [%d]", RANGE_DIFF);
}

bool LaserDetector::isNextToCorner() const {
  float laserDiffFront =
      currentLaserScan->ranges[RANGE_START - RANGE_DIFF * 2] -
      currentLaserScan->ranges[RANGE_START - RANGE_DIFF];
  float laserDiffBack = currentLaserScan->ranges[RANGE_START - RANGE_DIFF] -
                        currentLaserScan->ranges[RANGE_START];
  bool retVal = laserDiffFront > (1.3 / CORNER_SENSITIVITY) &&
                laserDiffBack < (0.2 * CORNER_SENSITIVITY);

  // ROS_INFO("Distance to Corner Front: [%f]", laserDiffFront);
  // ROS_INFO("Distance to Corner: [%f]", laserDiffBack);
  return retVal;
}

float LaserDetector::getFrontObstacleDist() const {
  float viewangle = currentLaserScan->angle_max - currentLaserScan->angle_min;
  int steps = (int)(viewangle / currentLaserScan->angle_increment);

  //float angle = getAngleToWall();

  int stepsLookingAway = 0; //(int) angle / currentLaserScan->angle_increment;
  
  float rightDist = currentLaserScan->ranges[(int)(steps/2 - RANGE_DIFF * 4 + stepsLookingAway)];
  float rightMidDist = currentLaserScan->ranges[(int)(steps/2 - RANGE_DIFF * 2 + stepsLookingAway)];
  float leftMidDist = currentLaserScan->ranges[(int)(steps/2 - RANGE_DIFF * 2 + stepsLookingAway)];
  float leftDist = currentLaserScan->ranges[(int)(steps/2 + RANGE_DIFF * 4 + stepsLookingAway)];
  float minRight = std::min(rightDist, rightMidDist); 
  float minLeft = std::min(leftMidDist, leftDist);
  float min = std::min(minRight, minLeft);
  return min;
}

bool LaserDetector::isObstacleRight() {
  float viewangle = currentLaserScan->angle_max - currentLaserScan->angle_min;
  int steps = (int)(viewangle / currentLaserScan->angle_increment);

  //float angle = getAngleToWall();

  int stepsLookingAway = 0; //(int) angle / currentLaserScan->angle_increment;
  
  float rightDist = currentLaserScan->ranges[(int)(steps/2 - RANGE_DIFF * 6 + stepsLookingAway)];
  float rightMidDist = currentLaserScan->ranges[(int)(steps/2 - RANGE_DIFF * 3 + stepsLookingAway)];
  float leftMidDist = currentLaserScan->ranges[(int)(steps/2 - RANGE_DIFF * 3 + stepsLookingAway)];
  float leftDist = currentLaserScan->ranges[(int)(steps/2 + RANGE_DIFF * 6 + stepsLookingAway)];
  float minRight = std::min(rightDist, rightMidDist); 
  float minLeft = std::min(leftMidDist, leftDist);
  return minRight < minLeft;
}

float LaserDetector::getDistanceToWall() const {
  //  float angle_range =
  //    currentLaserScan->angle_max - currentLaserScan->angle_min; // 70 deg
  float alpha =
      RANGE_DIFF *
      currentLaserScan
          ->angle_increment; // angle alpha used for Calculation in rad

  float beta1 = this->calculateBeta(RANGE_START, alpha);
  float beta2 =
      this->calculateBeta(RANGE_START - ((int)RANGE_DIFF * 0.333), alpha);
  float beta3 =
      this->calculateBeta(RANGE_START - ((int)RANGE_DIFF * 0.666), alpha);
  float beta4 =
      this->calculateBeta(RANGE_START - ((int)RANGE_DIFF * 1.0), alpha);
  float beta5 =
      this->calculateBeta(RANGE_START - ((int)RANGE_DIFF * 1.333), alpha);

  float beta = (beta1 + beta2 + beta3 + beta4 + beta5) / 5.0;
  float distance =
      sin(beta) * currentLaserScan->ranges[RANGE_START - RANGE_DIFF];
  // ROS_INFO("Distance to Wall: [%f]", distance);
  return distance;
}

float LaserDetector::getDistanceToCorner() const {
  float distance = currentLaserScan->ranges[RANGE_START];
  distance = distance * getAngleToWall();
  ROS_INFO("Distance to Corner: [%f]", distance);
  return distance;
}

bool LaserDetector::isNextToWall() const {
  if (this->isNextToCorner())
    return false;

  float laserDiffFront =
      currentLaserScan->ranges[RANGE_START - RANGE_DIFF * 3] -
      currentLaserScan->ranges[RANGE_START - RANGE_DIFF * 2];
  float laserDiffBack = currentLaserScan->ranges[RANGE_START - RANGE_DIFF * 1] -
                        currentLaserScan->ranges[RANGE_START - RANGE_DIFF * 2];
  bool retVal = currentLaserScan->ranges[RANGE_START - RANGE_DIFF * 3] < 3.0 &&
                currentLaserScan->ranges[RANGE_START - RANGE_DIFF * 2] < 2.8 &&
                currentLaserScan->ranges[RANGE_START - RANGE_DIFF * 1] < 2.6 &&
                laserDiffFront < 0.7 && laserDiffBack < 0.5;
  return retVal;
}

float LaserDetector::calculateBeta(int angleBegin, float &alpha) const {

  float b = currentLaserScan->ranges[angleBegin];
  float c = currentLaserScan->ranges[angleBegin - RANGE_DIFF];
  float a = sqrt(b * b + c * c - 2 * b * c * cos(alpha));
  float beta = asin(b * sin(alpha) / a);

  // float angleToWall = PI - beta - epsilon;
  // ROS_INFO("Angle to wall in deg: [%f]", angleToWall * 180 / PI);
  return beta;
}

void LaserDetector::setCurrentLaserScan(
    const sensor_msgs::LaserScanConstPtr &value) {
  currentLaserScan = value;
}

float LaserDetector::getAngleToWall() const {
  float angle_range =
      currentLaserScan->angle_max - currentLaserScan->angle_min; // 70 deg
  float alpha =
      RANGE_DIFF *
      currentLaserScan
          ->angle_increment; // angle alpha used for Calculation in rad
  float epsilon = PI / 2 - angle_range / 2 + alpha;

  float beta1 = this->calculateBeta(RANGE_START, alpha);
  float beta2 =
      this->calculateBeta(RANGE_START - ((int)RANGE_DIFF * 0.333), alpha);
  float beta3 =
      this->calculateBeta(RANGE_START - ((int)RANGE_DIFF * 0.666), alpha);
  float beta4 =
      this->calculateBeta(RANGE_START - ((int)RANGE_DIFF * 1.0), alpha);
  float beta5 =
      this->calculateBeta(RANGE_START - ((int)RANGE_DIFF * 1.333), alpha);

  float beta = (beta1 + beta2 + beta3 + beta4 + beta5) / 5.0;

  // ROS_INFO("beta in deg: [%f]", beta * 180 / PI);

  float angleToWall = PI - beta - epsilon - ANGLE_OFFSET;
  // ROS_INFO("Angle to wall in deg: [%f]", angleToWall * 180 / PI);
  return angleToWall;
}

float LaserDetector::getAngleToWallInDeg() const {
  float angleToWall = this->getAngleToWall();
  return (angleToWall * 180.0 / PI);
}

#endif
