#include "autu_control/emergency_brake.h"
#include <limits>
#include <math.h>

#define DURATION 0.1

#define DRIVE 0
#define STOP 1

EmergencyBrake::EmergencyBrake(ros::NodeHandle *n)
    : maxMotorLevel(999), us_front(2), carWidth(0.2), distanceToObstacle(10.0),
      state(DRIVE) {

  n->param<float>("deceleration", deceleration, 0.9);
  n->param<float>("safety_distance", safetyDistance, 0.1);

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

  sensor_sub = n->subscribe<pses_basis::SensorData>(
      "pses_basis/sensor_data", 1,
      std::bind(&EmergencyBrake::sensorDataCallback, this,
                std::placeholders::_1));

  laserscan_sub = n->subscribe<sensor_msgs::LaserScan>(
      "/scan", 10, std::bind(&EmergencyBrake::laserscanCallback, this,
                             std::placeholders::_1));

  odom_sub = n->subscribe<nav_msgs::Odometry>(
      "/odom", 1,
      std::bind(&EmergencyBrake::odomCallback, this, std::placeholders::_1));
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
  // currentSpeed = msg->speed;
}

void EmergencyBrake::sensorDataCallback(
    const pses_basis::SensorDataConstPtr &msg) {
  us_front = msg->range_sensor_front;
  if (!std::isnan(msg->hall_sensor_dt_full) && msg->hall_sensor_dt_full != 0)
    currentSpeed = 0.2 / msg->hall_sensor_dt_full;
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

    // camera isn't at the car's front
    d_min -= 0.1;
    if (d_min < 0)
      d_min = 0;
    else if (d_min > laserscan->range_max)
      d_min = laserscan->range_max;

    if (us_front < 0.5 && 0 < us_front && us_front < d_min)
      distanceToObstacle = us_front;
    else
      distanceToObstacle = d_min;
  }
}

void EmergencyBrake::laserscanCallback(
    const sensor_msgs::LaserScanConstPtr &msg) {
  laserscan = msg;
}

void EmergencyBrake::timerCallback(const ros::TimerEvent &) {
  updateDistanceToObstacle();
  const float maxSpeed =
      std::sqrt(2 * deceleration * (distanceToObstacle + safetyDistance));
  uint8_t nextState = DRIVE;
  switch (state) {
  case DRIVE:
    if (currentSpeed > maxSpeed) {
      maxMotorLevel = 0;

      pses_basis::Command stopCmd;
      stopCmd.header.stamp = ros::Time::now();
      stopCmd.motor_level = 0;
      command_pub.publish(stopCmd);

      nextState = STOP;
    }
    break;
  case STOP:
    if (currentSpeed < 0.8 * maxSpeed) {
      // maxMotorLevel = 999;
      // nextState = DRIVE;
    }
    break;
  default:
    break;
  }
  state = nextState;

  if (maxMotorLevel <= 3)
    maxMotorLevel = 0;
  ROS_INFO("usf: %f dist: %f cur_speed: %f max_speed: %f maxLevel: %d",
           us_front, distanceToObstacle, currentSpeed, maxSpeed, maxMotorLevel);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "emergency_brake");
  ros::NodeHandle n;
  EmergencyBrake emergencyBrake(&n);
  ROS_INFO("Emergency Brake launched");

  ros::spin();
  return 0;
}
