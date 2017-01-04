#pragma once

#include <cmath>
#include <vector>

class Laserscan {
public:
  Laserscan() {
    angle_min = -M_PI / 2;
    angle_max = M_PI / 2;
    range_min = 0;
    range_max = 10;
  }
  std::vector<float> ranges;
  float angle_min;       // start angle of the scan [rad]
  float angle_max;       // end angle of the scan [rad]
  float angle_increment; // angular distance between measurements [rad]
  float range_min;       // minimum range value [m]
  float range_max;       // maximum range value [m]
};