#include "autu_control/emergency_brake.h"

EmergencyBrake::EmergencyBrake(ros::NodeHandle *n)
    : maxMotorLevel(999), gridId(0), carX(0), carY(0) {
  command_pub = n->advertise<pses_basis::Command>("pses_basis/command", 10);

  map_pub = n->advertise<nav_msgs::OccupancyGrid>("autu/emergency_map", 10);

  timer = n->createTimer(
      ros::Duration(1.0),
      std::bind(&EmergencyBrake::timerCallback, this, std::placeholders::_1));

  command_sub = n->subscribe<pses_basis::Command>(
      "autu/command", 10,
      std::bind(&EmergencyBrake::commandCallback, this, std::placeholders::_1));

  speed_sub = n->subscribe<pses_basis::CarInfo>(
      "pses_basis/car_info", 10,
      std::bind(&EmergencyBrake::speedCallback, this, std::placeholders::_1));

  map_sub = n->subscribe<nav_msgs::OccupancyGrid>(
      "autu/gridmap", 10,
      std::bind(&EmergencyBrake::gridMapCallback, this, std::placeholders::_1));

  odom_sub = n->subscribe<nav_msgs::Odometry>(
      "/odom", 10,
      std::bind(&EmergencyBrake::odomCallback, this, std::placeholders::_1));
}

EmergencyBrake::~EmergencyBrake() { command_sub.shutdown(); }

void EmergencyBrake::gridMapCallback(
    const nav_msgs::OccupancyGridConstPtr &msg) {
  grid = msg;
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

void EmergencyBrake::odomCallback(const nav_msgs::OdometryConstPtr &msg) {
  carX = msg->pose.pose.position.x;
  carY = msg->pose.pose.position.y;
}

void EmergencyBrake::timerCallback(const ros::TimerEvent &) {
  ROS_INFO("Timer Callback");
  if (grid != nullptr) {
    nav_msgs::OccupancyGrid debugGrid = *grid;
    //
    int x =
        (carX - debugGrid.info.origin.position.x) / debugGrid.info.resolution;
    int y =
        (carY - debugGrid.info.origin.position.y) / debugGrid.info.resolution;
    // check array bounds

    /*debugGrid.header.seq = gridId++;
    debugGrid.header.stamp = ros::Time::now();
    debugGrid.header.frame_id = "map";
    debugGrid.info.resolution = 0.05;
    debugGrid.info.height = 50;
    debugGrid.info.width = 100;
    debugGrid.info.origin.orientation.w = 1;*/
    // for (uint32_t i = 0; i < debugGrid.info.width * debugGrid.info.height;
    // ++i)
    const int pos = y * debugGrid.info.width + x;
    if (pos < debugGrid.data.size())
      debugGrid.data[pos] = 100;
    map_pub.publish(debugGrid);
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
