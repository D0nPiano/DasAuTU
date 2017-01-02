#include "autu_control/rundkurs/laser_utilities.h"

#include <vector>

typedef struct {
  float x;
  float y;
} Point;

float autu::getDistanceToWall(const sensor_msgs::LaserScanConstPtr &scan,
                              bool left) {
  if (scan == nullptr)
    return -1;
  else if (left) {
    /* size_t i;
     for (i = laserscan->ranges.size() - 1; i > laserscan->ranges.size() / 2;
          --i) {
       const float r = laserscan->ranges[i];
       if (laserscan->range_min < r && r < laserscan->range_max) {
         if (r - last_r > 1.0)
           break;
         else
           last_r = r;
       }
     }
     const float alpha =
         std::abs(laserscan->angle_max - (i + 1) * laserscan->angle_increment);
     corner.x = last_r * std::cos(alpha);
     corner.y = last_r * std::sin(alpha);*/
  }
  return 0;
}
