#include "autu_control/rundkurs/laser_utilities.h"

#include <cmath>
#include <ctime>
#include <limits>
#include <stdlib.h>

#include "geometry_msgs/Pose.h"
#include "nav_msgs/Path.h"

#define MAX_WALL_SCOPE 2.0f
#define MAX_WALL_DIST 1.0f

using std::sqrt;
using std::min;
using std::cos;
using std::sin;
using std::abs;
using std::vector;

using Eigen::Vector2f;
using Eigen::ParametrizedLine;

LaserUtil::LaserUtil(ros::NodeHandle &nh) {
  wall_pub = nh.advertise<nav_msgs::Path>("autu/wall", 1);
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

float LaserUtil::getDistanceToWall(const sensor_msgs::LaserScanConstPtr &scan,
                                   bool left) {
  if (scan == nullptr)
    return -1;
  else {
    const float r_max =
        min(scan->range_max, sqrt(MAX_WALL_DIST * MAX_WALL_DIST +
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
