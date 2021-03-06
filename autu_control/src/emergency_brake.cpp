#include "autu_control/emergency_brake.h"
#include <limits>
#include <math.h>

#include "std_msgs/Float32.h"

#define DRIVEN_DISTANCE_PER_TICK 0.0251327412 // RAD_PER_TICK * WHEEL_RADIUS

#define DRIVE 0
#define STOP 1

EmergencyBrake::EmergencyBrake(ros::NodeHandle *n)
    : maxMotorLevel(999), speedCarInfo(0), speedTimestamp(0), us_front(2),
      carWidth(0.2), distanceToObstacle(10.0), deceleration(0.9),
      safetyDistance(0.1), state(DRIVE), lastSteering(0) {

  n->getParam("/emergency_brake/deceleration", deceleration);
  n->getParam("/emergency_brake/safety_distance", safetyDistance);
  duration = n->param<float>("/emergency_brake/duration", 0.01);

  command_pub = n->advertise<pses_basis::Command>("pses_basis/command", 3);

  // debug_pub = n->advertise<std_msgs::Float32>("autu/speed", 3);

  timer = n->createTimer(
      ros::Duration(duration),
      std::bind(&EmergencyBrake::timerCallback, this, std::placeholders::_1));

  command_sub = n->subscribe<pses_basis::Command>(
      "autu/command", 10,
      std::bind(&EmergencyBrake::commandCallback, this, std::placeholders::_1));

  sensor_sub = n->subscribe<pses_basis::SensorData>(
      "pses_basis/sensor_data", 1,
      std::bind(&EmergencyBrake::sensorDataCallback, this,
                std::placeholders::_1));

  laserscan_sub = n->subscribe<sensor_msgs::LaserScan>(
      "/scan", 10, std::bind(&EmergencyBrake::laserscanCallback, this,
                             std::placeholders::_1));

  carinfo_sub = n->subscribe<pses_basis::CarInfo>(
      "pses_basis/car_info", 1,
      std::bind(&EmergencyBrake::carInfoCallback, this, std::placeholders::_1));
}

EmergencyBrake::~EmergencyBrake() {
  command_sub.shutdown();
  laserscan_sub.shutdown();
  sensor_sub.shutdown();
}

void EmergencyBrake::carInfoCallback(const pses_basis::CarInfoConstPtr &msg) {
  speedCarInfo = msg->speed;
}

void EmergencyBrake::commandCallback(const pses_basis::CommandConstPtr &cmd) {
  // copy message to avoid crazy shit
  pses_basis::Command replacement = *cmd;

  // limit motor level
  if (replacement.motor_level > maxMotorLevel)
    replacement.motor_level = maxMotorLevel;

  // remember steering level
  lastSteering = replacement.steering_level;

  // publish new command on output topic
  command_pub.publish(replacement);
}

void EmergencyBrake::sensorDataCallback(
    const pses_basis::SensorDataConstPtr &msg) {
  us_front = msg->range_sensor_front;
  if (!std::isnan(msg->hall_sensor_dt)) {
    currentSpeed = DRIVEN_DISTANCE_PER_TICK / msg->hall_sensor_dt;
  }
}

void EmergencyBrake::updateDistanceToObstacle() {
  if (laserscan != nullptr) {
    float d_min = std::numeric_limits<float>::max();

    //calculates distance to nearest object in sight of Kinect
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

    distanceToObstacle -= safetyDistance;
    if (distanceToObstacle < 0)
      distanceToObstacle = 0;
  }
}

void EmergencyBrake::laserscanCallback(
    const sensor_msgs::LaserScanConstPtr &msg) {
  laserscan = msg;
}

void EmergencyBrake::timerCallback(const ros::TimerEvent &) {
  updateDistanceToObstacle();
  const float maxSpeed = std::sqrt(2 * deceleration * distanceToObstacle);
  switch (state) {
  case DRIVE:
    if (currentSpeed > maxSpeed) {
      pses_basis::Command stopCmd;
      stopCmd.header.stamp = ros::Time::now();
      stopCmd.motor_level = 0;
      stopCmd.steering_level = lastSteering;
      command_pub.publish(stopCmd);

      state = STOP;
    }
    break;
  case STOP:
    if (currentSpeed < 0.5 * maxSpeed) {
      state = DRIVE;
    }
    break;
  default:
    break;
  }

  switch (state) {
  case DRIVE:
    maxMotorLevel = 999;
    break;
  case STOP:
    maxMotorLevel = 0;
    break;
  default:
    break;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "emergency_brake");
  ros::NodeHandle n;
  EmergencyBrake emergencyBrake(&n);
  ROS_INFO("Emergency Brake launched");

  ros::spin();
  return 0;
}
