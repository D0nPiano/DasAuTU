#include "autu_control/parking/corner.h"

#include <cmath>

using std::fmin;
using std::fmax;
using Eigen::ParametrizedLine;
using Eigen::Vector2f;

Corner::Corner() {}

Corner::Corner(const Eigen::Vector2f &origin,
               const Eigen::Vector2f &direction_x,
               const Eigen::Vector2f &direction_y)
    : origin(origin), direction_x(direction_x), direction_y(direction_y),
      x_line(origin, direction_x), y_line(origin, direction_y) {}

float Corner::distance(const Eigen::Vector2f &point) const {
  const Vector2f p = point - origin;

  const float dot_x = p.dot(direction_x);
  const float dot_y = p.dot(direction_y);

  if (dot_x > 0 && dot_y > 0) {
    const float dist_x = x_line.distance(point);
    const float dist_y = y_line.distance(point);
    return fmin(dist_x, dist_y);
  } else if (dot_x > 0)
    return x_line.distance(point);
  else if (dot_y > 0)
    return y_line.distance(point);
  else {
    // dot_x < 0 && dot_y < 0
    const float dist_x = x_line.distance(point);
    const float dist_y = y_line.distance(point);
    return fmax(dist_x, dist_y);
  }
}

nav_msgs::Path Corner::toPath1() const {
  nav_msgs::Path msg;
  for (int i = 0; i < 5; ++i) {
    const Vector2f &p = x_line.pointAt(i);
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = p[0];
    pose.pose.position.y = p[1];
    msg.poses.push_back(pose);
  }
  msg.header.frame_id = "base_laser";
  return msg;
}

nav_msgs::Path Corner::toPath2() const {
  nav_msgs::Path msg;
  for (int i = 0; i < 5; ++i) {
    const Vector2f &p = y_line.pointAt(i);
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = p[0];
    pose.pose.position.y = p[1];
    msg.poses.push_back(pose);
  }
  msg.header.frame_id = "base_laser";
  return msg;
}

Eigen::Vector2f Corner::getOrigin() const { return origin; }
