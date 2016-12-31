#ifndef EMERGENCYBRAKE_H
#define EMERGENCYBRAKE_H

#include "laserscan.h"

class EmergencyBrake {
public:
  EmergencyBrake();
  ~EmergencyBrake();
  void updateDistanceToObstacle();

  const Laserscan *laserscan;
  float distanceToObstacle;

private:
  float currentSpeed;
  float speedCarInfo;
  double speedTimestamp;
  float us_front;
  float carWidth;
  float deceleration, safetyDistance;
};

#endif // EMERGENCYBRAKE_H
