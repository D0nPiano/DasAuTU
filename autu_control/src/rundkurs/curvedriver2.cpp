#include "autu_control/rundkurs/curvedriver2.h"

#include <math.h>

#include "pses_basis/Command.h"

#define MIN_RADIUS 0.6
#define MAX_STEERING_ANGLE 20.0
#define MAX_STEERING_LEVEL 40.0
#define WHEELBASE 0.28

CurveDriver2::CurveDriver2(ros::NodeHandle &nh) : e0(0), t0(0), radius(1) {
  command_pub = nh.advertise<pses_basis::Command>("autu/command", 1);
}

void CurveDriver2::reset() {}

void CurveDriver2::drive(const nav_msgs::OdometryConstPtr &odom) {
  const float xDif = rotationCenter.position.x - odom->pose.pose.position.x;
  const float yDif = rotationCenter.position.y - odom->pose.pose.position.y;
  // actual distance
  const float currentRadius = std::sqrt(xDif * xDif + yDif * yDif);

  pses_basis::Command cmd;

  float solldist = radius;
  float steerfact = -2;

  // P-Regler, tb = 62s

  cmd.motor_level = 8;
  float p = 16;
  float d = 8;
  double t = ros::Time::now().toSec();
  float e = solldist - currentRadius;
  cmd.steering_level =
      initialSteering + steerfact * p * (e + (e - e0) * d / (t - t0));

  e0 = e;
  t0 = t;

  if (cmd.steering_level > 40)
    cmd.steering_level = 40;
  else if (cmd.steering_level < -40)
    cmd.steering_level = -40;

  cmd.header.stamp = ros::Time::now();
  command_pub.publish(cmd);
}

bool CurveDriver2::isAroundTheCorner() const { return false; }

void CurveDriver2::curveInit(float radius, bool left,
                             const nav_msgs::OdometryConstPtr &odom) {
  this->radius = radius;
  rotationCenter.position = odom->pose.pose.position;

  if (left)
    rotationCenter.position.y += radius;
  else
    rotationCenter.position.y -= radius;
  const float alpha = M_PI / 2 - std::atan(radius / WHEELBASE);
  initialSteering = alpha / MAX_STEERING_ANGLE * MAX_STEERING_LEVEL;

  initialSteering = 2 * 15.642;
  if (!left)
    initialSteering = -initialSteering;
}
