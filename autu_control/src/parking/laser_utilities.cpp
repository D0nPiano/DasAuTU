#include "autu_control/parking/laser_utilities.h"

#include <cmath>
#include <ctime>
#include <limits>
#include <stdlib.h>

#include "nav_msgs/Path.h"

using std::sqrt;
using std::fmin;
using std::fmax;
using std::cos;
using std::sin;
using std::abs;
using std::vector;

using Eigen::Vector2f;
using Eigen::ParametrizedLine;
using Eigen::aligned_allocator;

LaserUtil::LaserUtil(ros::NodeHandle &nh) {
  corner1Pub = nh.advertise<nav_msgs::Path>("autu/debug/corner_1", 1);
  corner2Pub = nh.advertise<nav_msgs::Path>("autu/debug/corner_2", 1);

  deltaMax = nh.param<float>("main/laser_util/max_wall_scope", 0.1f);
  minimumLineSize = nh.param<int>("main/laser_util/minimum_line_size", 6);
  maxWallScope = nh.param<float>("main/laser_util/max_wall_scope", 1.5f);
  maxWallDistance = nh.param<float>("main/laser_util/max_wall_distance", 0.5f);
}

Vector2f LaserUtil::findCorner(const sensor_msgs::LaserScanConstPtr &scan) {
  if (scan == nullptr)
    return Vector2f(0, 0);

  // publish the detected corner
  Corner corner = findCornerRLF(convertToCartesian(scan, false));
  corner1Pub.publish(corner.toPathMsg1());
  corner2Pub.publish(corner.toPathMsg2());

  return corner.getB();
}

Corner LaserUtil::findCornerRLF(
    const vector<Vector2f, aligned_allocator<Vector2f>> &points) {
  const vector<Line> &lines = findLinesRLF(points);

  // A corner consists of 2 lines
  if (lines.size() >= 2) {

    Line lastLine = lines.back();

    // Examine two adjacent lines. If they have a common point they make up a
    // corner.
    for (auto iter = lines.rbegin() + 1; iter != lines.rend(); ++iter)

      if (lastLine.hasPointInCommon(*iter))
        // Two lines have a common point -> a corner was found
        return Corner(lastLine, *iter);
      else
        // Check the next pair of lines
        lastLine = *iter;
  }

  // No corner detected
  return Corner();
}

vector<Line> LaserUtil::findLinesRLF(
    const vector<Vector2f, aligned_allocator<Vector2f>> &points) {
  vector<Line> lines;

  // too little amount of points to form a line
  if (points.size() < minimumLineSize)
    return lines;

  const Vector2f &first = points.front();
  const Vector2f &last = points.back();

  // line through the first and the last point
  ParametrizedLine<float, 2> line =
      ParametrizedLine<float, 2>::Through(first, last);

  // find the point with the biggest distance to that line
  float maxError = 0;
  size_t maxErrorIndex;
  size_t index = 0;

  for (const Vector2f &point : points) {
    const float distance = line.distance(point);
    if (distance > maxError) {
      maxError = distance;
      maxErrorIndex = index;
    }
    ++index;
  }

  // if the maiximum error is bigger than the threshold split the points
  if (maxError > deltaMax) {
    // split the line at the point with the maximum error
    vector<Vector2f, aligned_allocator<Vector2f>> data;

    // find lines in the first half
    for (size_t i = 0; i <= maxErrorIndex; ++i)
      data.push_back(points[i]);
    vector<Line> result = findLinesRLF(data);

    lines.insert(lines.end(), result.begin(), result.end());

    data.clear();
    // find lines in the second half
    for (size_t i = maxErrorIndex; i < points.size(); ++i)
      data.push_back(points[i]);
    result = findLinesRLF(data);

    lines.insert(lines.end(), result.begin(), result.end());

  } else
    // line was found
    lines.push_back(Line(first, last));

  return lines;
}

vector<Vector2f, aligned_allocator<Vector2f>>
LaserUtil::convertToCartesian(const sensor_msgs::LaserScanConstPtr &scan,
                              bool left) {
  // ignore points which are far away
  const float r_max =
      fmin(scan->range_max, sqrt(maxWallDistance * maxWallDistance +
                                 maxWallScope * maxWallScope));

  vector<Vector2f, aligned_allocator<Vector2f>> points;

  // Determine which part of the scan should be used
  size_t lowerBound, upperBound;
  if (left) {
    lowerBound = scan->ranges.size() / 2;
    upperBound = scan->ranges.size() - 1;
  } else {
    // right
    lowerBound = 0;
    upperBound = scan->ranges.size() / 2;
  }

  // iterate over one half of the scan
  for (size_t i = lowerBound; i < upperBound; ++i) {
    const float r = scan->ranges[i];
    // check if point is valid
    if (scan->range_min < r && r < r_max) {
      const float alpha = scan->angle_min + i * scan->angle_increment;
      // convert to cartesian
      Vector2f point;
      point[0] = r * cos(alpha);
      point[1] = r * sin(alpha);
      points.push_back(point);
    }
  }

  return points;
}
