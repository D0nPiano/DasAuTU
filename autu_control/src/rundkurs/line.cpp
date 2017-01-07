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
