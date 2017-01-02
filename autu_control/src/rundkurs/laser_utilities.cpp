#include "autu_control/rundkurs/laser_utilities.h"

#include <cmath>
#include <vector>

#define MAX_WALL_SCOPE 1.5f
#define MAX_WALL_DIST 1.0f

using std::sqrt;
using std::min;
using std::cos;
using std::sin;
using std::abs;
using std::vector;

typedef struct {
  float x;
  float y;
} Point;

float autu::getDistanceToWall(const sensor_msgs::LaserScanConstPtr &scan,
                              bool left) {
  if (scan == nullptr)
    return -1;
  else {
    const float r_max =
        min(scan->range_max, sqrt(MAX_WALL_DIST * MAX_WALL_DIST +
                                  MAX_WALL_SCOPE * MAX_WALL_SCOPE));
    size_t i;
    vector<Point> points;
    if (left) {
      for (i = scan->ranges.size() - 1; i > scan->ranges.size() / 2; --i) {
        const float r = scan->ranges[i];
        if (scan->range_min < r && r < r_max) {
          const float alpha = abs(scan->angle_min + i * scan->angle_increment);
          Point point;
          point.x = r * cos(alpha);
          point.y = r * sin(alpha);
          points.push_back(point);
        }
      }
    }
  }
  return 0;
}
