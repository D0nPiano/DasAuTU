#include "autu_control/parking/line.h"

using Eigen::Vector2f;
using Eigen::ParametrizedLine;

Line::Line(const Vector2f &first, const Vector2f &last)
    : origin(first), end(last) {}

nav_msgs::Path Line::toPathMsg() const {
  nav_msgs::Path msg;
  geometry_msgs::PoseStamped pose;

  pose.pose.position.x = origin[0];
  pose.pose.position.y = origin[1];
  msg.poses.push_back(pose);

  pose.pose.position.x = end[0];
  pose.pose.position.y = end[1];
  msg.poses.push_back(pose);

  msg.header.frame_id = "base_laser";
  return msg;
}

bool Line::hasPointInCommon(const Line &other) const {
  return origin == other.origin || origin == other.end || end == other.origin ||
         end == other.end;
}

ParametrizedLine<float, 2> Line::toEigenLine() const {
  return ParametrizedLine<float, 2>::Through(origin, end);
}

Vector2f Line::getOrigin() const { return origin; }

Vector2f Line::getEnd() const { return end; }
