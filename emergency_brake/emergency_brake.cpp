#include "emergency_brake.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <omp.h>

#define DURATION 0.1

#define DRIVE 0
#define STOP 1

using std::cout;
using std::endl;

EmergencyBrake::EmergencyBrake()
    : distanceToObstacle(10.0),speedCarInfo(0), speedTimestamp(0), us_front(2), carWidth(0.2),
       deceleration(0.9), safetyDistance(0.1) {}

EmergencyBrake::~EmergencyBrake() {}

void EmergencyBrake::updateDistanceToObstacle() {
  if (laserscan != nullptr) {
    float d_min = std::numeric_limits<float>::max();

    for (size_t i = 0; i < laserscan->ranges.size(); ++i) {
      const float r = laserscan->ranges[i];
      if (laserscan->range_min < r && r < laserscan->range_max) {
        // alpha in radians and always positive
        const float alpha =
            std::abs(i * laserscan->angle_increment + laserscan->angle_min);
        const float sin_alpha = std::sin(alpha);
        const float b = carWidth / 2.0;
        if (sin_alpha != 0.0 && r < b / sin_alpha) {
          // distance to obstacle
          const float d = r * std::cos(alpha);
          if (d < d_min)
            d_min = d;
        }
      }
    }

    // camera isn't at the car's front
    d_min -= 0.1;
    if (d_min < 0)
      d_min = 0;
    else if (d_min > laserscan->range_max)
      d_min = laserscan->range_max;

    if (us_front < 0.5 && 0 < us_front && us_front < d_min)
      distanceToObstacle = us_front;
    else
      distanceToObstacle = d_min;

    distanceToObstacle -= safetyDistance;
    if (distanceToObstacle < 0)
      distanceToObstacle = 0;
  }
}

int main(int argc, char **argv) {
  EmergencyBrake emergencyBrake;
  Laserscan scan;
  const int scanpoints = 200;
  const int iterations = 1000000;
  scan.angle_increment = M_PI / (scanpoints - 1);
  for (int i = 0; i < scanpoints; ++i) {
    scan.ranges.push_back(3);
  }
  emergencyBrake.laserscan = &scan;
  double time = -omp_get_wtime();
  for (long i = 0; i < iterations; ++i)
    emergencyBrake.updateDistanceToObstacle();
  time += omp_get_wtime();
  cout << "nicht optimiert: " << time / iterations << " Sekunden pro Aufruf"
       << endl;
  return 0;
}
