#include "autu_control/parking/corner.h"

#include <cmath>

using std::fmin;
using std::fmax;
using Eigen::ParametrizedLine;
using Eigen::Vector2f;

Corner::Corner(const Vector2f &a, const Vector2f &b, const Vector2f &c)
    : a(a), b(b), c(c) {}

Corner::Corner(const Line &line1, const Line &line2) {
  if (line1.getOrigin() == line2.getOrigin()) {
    a = line1.getEnd();
    b = line1.getOrigin();
    c = line2.getEnd();
  } else if (line1.getOrigin() == line2.getEnd()) {
    a = line1.getEnd();
    b = line1.getOrigin();
    c = line2.getOrigin();
  } else if (line1.getEnd() == line2.getOrigin()) {
    a = line1.getOrigin();
    b = line1.getEnd();
    c = line2.getEnd();
  } else if (line1.getEnd() == line2.getEnd()) {
    a = line1.getOrigin();
    b = line1.getEnd();
    c = line2.getOrigin();
  }
}

nav_msgs::Path Corner::toPathMsg1() const {
  nav_msgs::Path msg;
  geometry_msgs::PoseStamped pose;
  pose.pose.position.z = -0.2f;
  pose.pose.position.x = b[0];
  pose.pose.position.y = b[1];
  msg.poses.push_back(pose);

  pose.pose.position.x = a[0];
  pose.pose.position.y = a[1];
  msg.poses.push_back(pose);

  msg.header.frame_id = "base_laser";
  return msg;
}
nav_msgs::Path Corner::toPathMsg2() const {
  nav_msgs::Path msg;
  geometry_msgs::PoseStamped pose;
  pose.pose.position.z = -0.2f;

  pose.pose.position.x = b[0];
  pose.pose.position.y = b[1];
  msg.poses.push_back(pose);

  pose.pose.position.x = c[0];
  pose.pose.position.y = c[1];
  msg.poses.push_back(pose);

  msg.header.frame_id = "base_laser";
  return msg;
}

Vector2f Corner::getB() const { return b; }
