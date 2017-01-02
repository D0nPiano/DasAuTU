#ifndef LASER_UTILITIES_H
#define LASER_UTILITIES_H

#include "sensor_msgs/LaserScan.h"

namespace autu {
float getDistanceToWall(const sensor_msgs::LaserScanConstPtr &scan, bool left);
}

#endif // LASER_UTILITIES_H
