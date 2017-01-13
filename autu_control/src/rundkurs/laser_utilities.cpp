#include "autu_control/rundkurs/laser_utilities.h"

#include <cmath>
#include <ctime>
#include <limits>
#include <stdlib.h>

#include "geometry_msgs/Pose.h"
#include "nav_msgs/Path.h"

#define MAX_WALL_SCOPE 1.5f
#define MAX_WALL_DIST 0.5f

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

  delta_max = 0.1f;

  wall_pub = nh.advertise<nav_msgs::Path>("autu/wall", 1);
#ifndef NDEBUG
  corner1_pub = nh.advertise<nav_msgs::Path>("autu/debug/corner_1", 1);
  corner2_pub = nh.advertise<nav_msgs::Path>("autu/debug/corner_2", 1);
#endif
}

Vector2f LaserUtil::findCorner(const sensor_msgs::LaserScanConstPtr &scan) {
  if (scan == nullptr)
    return Vector2f(0, 0);
  else
#ifndef NDEBUG
  {
    Corner corner = findCornerRLF(filterScan(scan, false));
    corner1_pub.publish(corner.toPathMsg1());
    corner2_pub.publish(corner.toPathMsg2());
    return corner.getB();
  }
#else
    return findCornerRLF(filterScan(scan, false)).getB();
#endif
}

ParametrizedLine<float, 2> LaserUtil::findLine(
    const std::vector<Eigen::Vector2f,
                      Eigen::aligned_allocator<Eigen::Vector2f>> &points) {
  std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>> data =
      points;  // a set of observed data points
  int k = 100; // the maximum number of iterations allowed in the algorithm
  float t =
      0.1f; // a threshold value for determining when a data point fits a model
  size_t d = 0.5 * data.size(); // the number of close data values required to
                                // assert that a model fits
                                // well to data
  ParametrizedLine<float, 2> bestfit;
  double besterr = std::numeric_limits<double>::max();
  std::srand(std::time(0));
  for (int iter = 0; iter < k; ++iter) {
    // determine two random points
    int o = (rand() % (int)(data.size() + 1));
    int dir = (rand() % (int)(data.size() + 1));
    Vector2f origin = data[o];
    while (dir == o)
      dir = (rand() % (int)(data.size() + 1));
    Vector2f direction = data[dir];

    ParametrizedLine<float, 2> maybe_model(
        origin,
        direction - origin); // model parameters fitted to maybeinliers
    std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>
        also_inliers; // empty set

    double error = 0;
    for (auto &point : data) {
      const float dist = maybe_model.distance(point);
      if (dist < t) {
        also_inliers.push_back(point);
        error += dist;
      }
    }
    error /= also_inliers.size();
    if (also_inliers.size() > d)
      if (error < besterr) {
        besterr = error;
        bestfit = maybe_model;
      }
  }
  return bestfit;
}

Corner LaserUtil::findCornerRLF(
    const vector<Vector2f, aligned_allocator<Vector2f>> &points) {
  const vector<Line> &lines = findLinesRLF(points);

#ifndef NDEBUG
  ROS_INFO("Lines detected: %lu", lines.size());
#endif

  if (lines.size() >= 2) {
    Line lastLine = lines.back();
    for (auto iter = lines.rbegin() + 1; iter != lines.rend(); ++iter)
      if (lastLine.hasPointInCommon(*iter))
        return Corner(lastLine, *iter);
      else
        lastLine = *iter;
  }
  // No corner detected
  return Corner();
}

vector<Line> LaserUtil::findLinesRLF(
    const vector<Vector2f, aligned_allocator<Vector2f>> &points) {
  vector<Line> lines;
  if (points.size() < 6)
    return lines;
  const Vector2f &first = points.front();
  const Vector2f &last = points.back();

  // line through the first and the last point
  ParametrizedLine<float, 2> line =
      ParametrizedLine<float, 2>::Through(first, last);

  // find the point with the biggest distance to that line
  float max_error = 0;
  size_t max_error_index;
  size_t index = 0;
  for (const Vector2f &point : points) {
    const float distance = line.distance(point);
    if (distance > max_error) {
      max_error = distance;
      max_error_index = index;
    }
    ++index;
  }

  if (max_error > delta_max) {
    // split the line at the point with the maximum error
    vector<Vector2f, aligned_allocator<Vector2f>> data;

    for (size_t i = 0; i <= max_error_index; ++i)
      data.push_back(points[i]);
    vector<Line> result = findLinesRLF(data);
    lines.insert(lines.end(), result.begin(), result.end());

    data.clear();
    for (size_t i = max_error_index; i < points.size(); ++i)
      data.push_back(points[i]);
    result = findLinesRLF(data);
    lines.insert(lines.end(), result.begin(), result.end());

  } else
    lines.push_back(Line(first, last));

  return lines;
}

float LaserUtil::getDistanceToWall(const sensor_msgs::LaserScanConstPtr &scan,
                                   bool left) {
  if (scan == nullptr)
    return -1;
  else {
    const float r_max =
        fmin(scan->range_max, sqrt(MAX_WALL_DIST * MAX_WALL_DIST +
                                   MAX_WALL_SCOPE * MAX_WALL_SCOPE));
    size_t i;
    std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>
        points;
    if (left) {
      for (i = scan->ranges.size() - 1; i > scan->ranges.size() / 2; --i) {
        const float r = scan->ranges[i];
        if (scan->range_min < r && r < r_max) {
          const float alpha = abs(scan->angle_min + i * scan->angle_increment);
          Vector2f point;
          point[0] = r * cos(alpha);
          point[1] = r * sin(alpha);
          points.push_back(point);
        }
      }
    }
    if (points.size() > 10) {
      ParametrizedLine<float, 2> wall = findLine(points);
      nav_msgs::Path msg;

      for (int i = -10; i < 10; ++i) {
        const auto &p = wall.pointAt(i);
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = p[0];
        pose.pose.position.y = p[1];
        msg.poses.push_back(pose);
      }
      msg.header.frame_id = "base_laser";
      wall_pub.publish(msg);
      Vector2f leftCorner(0, 0);
      return wall.distance(leftCorner);
    } else
      return -1;
  }

  return -1;
}

float LaserUtil::getAngleToWallRLF(const sensor_msgs::LaserScanConstPtr &scan,
                                   bool left) {
  const vector<Line> &lines = findLinesRLF(filterScan(scan, left));
  if (lines.size() == 0)
    return M_PI_2;
  else {
    const Line &wall = lines.front();
    const Vector2f e_x(1, 0);
    return std::acos(wall.toEigenLine().direction().dot(e_x));
  }
}

std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>
LaserUtil::filterScan(const sensor_msgs::LaserScanConstPtr &scan, bool left) {
  const float r_max =
      fmin(scan->range_max, sqrt(MAX_WALL_DIST * MAX_WALL_DIST +
                                 MAX_WALL_SCOPE * MAX_WALL_SCOPE));
  size_t i;
  std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>
      points;

  if (left)
    for (i = scan->ranges.size() - 1; i > scan->ranges.size() / 2; --i) {
      const float r = scan->ranges[i];
      if (scan->range_min < r && r < r_max) {
        const float alpha = scan->angle_min + i * scan->angle_increment;
        Vector2f point;
        point[0] = r * cos(alpha);
        point[1] = r * sin(alpha);
        points.push_back(point);
      }
    }
  else
    // right
    for (i = 0; i < scan->ranges.size() / 2; ++i) {
      const float r = scan->ranges[i];
      if (scan->range_min < r && r < r_max) {
        const float alpha = scan->angle_min + i * scan->angle_increment;
        Vector2f point;
        point[0] = r * cos(alpha);
        point[1] = r * sin(alpha);
        points.push_back(point);
      }
    }

  return points;
}
