#include "autu_control/rundkurs/line.h"

Line::Line(const Eigen::Vector2f &first, const Eigen::Vector2f &last)
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

bool Line::hasPointInCommon(const Line &other) {
  return origin == other.origin || origin == other.end || end == other.origin ||
         end == other.end;
}

Eigen::Vector2f Line::getOrigin() const { return origin; }

Eigen::Vector2f Line::getEnd() const { return end; }
