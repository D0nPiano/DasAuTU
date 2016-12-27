#include "autu_control/emergency_brake.h"
#include <limits>
#include <math.h>

#define DURATION 0.1

EmergencyBrake::EmergencyBrake(ros::NodeHandle *n)
    : maxMotorLevel(999), carWidth(0.2), distanceToObstacle(10.0) {
  command_pub = n->advertise<pses_basis::Command>("pses_basis/command", 10);

  timer = n->createTimer(
      ros::Duration(DURATION),
      std::bind(&EmergencyBrake::timerCallback, this, std::placeholders::_1));

  command_sub = n->subscribe<pses_basis::Command>(
      "autu/command", 10,
      std::bind(&EmergencyBrake::commandCallback, this, std::placeholders::_1));

  speed_sub = n->subscribe<pses_basis::CarInfo>(
      "pses_basis/car_info", 10,
      std::bind(&EmergencyBrake::speedCallback, this, std::placeholders::_1));

  laserscan_sub = n->subscribe<sensor_msgs::LaserScan>(
      "/scan", 10, std::bind(&EmergencyBrake::laserscanCallback, this,
                             std::placeholders::_1));

  /*  odom_sub = n->subscribe<nav_msgs::Odometry>(
        "/odom", 10,
        std::bind(&EmergencyBrake::odomCallback, this,
     std::placeholders::_1));*/
}

EmergencyBrake::~EmergencyBrake() {
  command_sub.shutdown();
  speed_sub.shutdown();
  odom_sub.shutdown();
  laserscan_sub.shutdown();
}

void EmergencyBrake::commandCallback(const pses_basis::CommandConstPtr &cmd) {
  // copy message to avoid crazy shit
  pses_basis::Command replacement = *cmd;

  // limit motor level
  if (replacement.motor_level > maxMotorLevel)
    replacement.motor_level = maxMotorLevel;

  // publish new command on output topic
  command_pub.publish(replacement);
}

void EmergencyBrake::speedCallback(const pses_basis::CarInfoConstPtr &msg) {
  currentSpeed = msg->speed;
}

void EmergencyBrake::odomCallback(const nav_msgs::OdometryConstPtr &msg) {}

void EmergencyBrake::updateDistanceToObstacle() {
  if (laserscan != nullptr) {
    float d_min = std::numeric_limits<float>::max();

    for (size_t i = 0; i < laserscan->ranges.size(); ++i) {
      const float r = laserscan->ranges[i];
      if (laserscan->range_min < r && r < laserscan->range_max) {
        // alpha in radians and always positive
        const float alpha =
            std::abs(i * laserscan->angle_increment + laserscan->angle_min);
        const float sin_alpha = std::sin(alpha);
        const float b = carWidth / 2.0;
        if (sin_alpha != 0.0 && r < b / sin_alpha) {
          // distance to obstacle
          const float d = r * std::cos(alpha);
          if (d < d_min)
            d_min = d;
        }
      }
    }

    if (d_min > laserscan->range_max)
      d_min = laserscan->range_max;

    distanceToObstacle = d_min;
  }
}

void EmergencyBrake::laserscanCallback(
    const sensor_msgs::LaserScanConstPtr &msg) {
  laserscan = msg;
}

void EmergencyBrake::timerCallback(const ros::TimerEvent &) {
  updateDistanceToObstacle();
  maxMotorLevel = 10 * std::sqrt(2 * distanceToObstacle);
  ROS_INFO("dist: %f maxLevel: %d", distanceToObstacle, maxMotorLevel);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "emergency_brake");
  ros::NodeHandle n;
  EmergencyBrake emergencyBrake(&n);
  ROS_INFO("Emergency Brake launched");

  ros::spin();
  return 0;
}
