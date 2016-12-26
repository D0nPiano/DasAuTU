#include "autu_control/emergency_brake.h"
#include <limits>
#include <math.h>

#define DURATION 1.0

EmergencyBrake::EmergencyBrake(ros::NodeHandle *n)
    : maxMotorLevel(999), gridId(0), carX(0), carY(0), carWidth(0.2) {
  command_pub = n->advertise<pses_basis::Command>("pses_basis/command", 10);

  map_pub = n->advertise<nav_msgs::OccupancyGrid>("autu/emergency_map", 10);

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
  //  carX = msg->pose.pose.position.x;
  //  carY = msg->pose.pose.position.y;
}

void EmergencyBrake::occupyCell(nav_msgs::OccupancyGrid &gridmap,
                                const geometry_msgs::Point &point) {
  int x = (point.x - gridmap.info.origin.position.x) / gridmap.info.resolution;
  int y = (point.y - gridmap.info.origin.position.y) / gridmap.info.resolution;
  if (0 < x && x < gridmap.info.width)
    if (0 < y && y < gridmap.info.height) {
      const int pos = y * gridmap.info.width + x;
      if (pos < gridmap.data.size())
        gridmap.data[pos] = 100;
    }
}

void EmergencyBrake::laserscanCallback(
    const sensor_msgs::LaserScanConstPtr &msg) {
  float d_min = std::numeric_limits<float>::max();
  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    const float r = msg->ranges[i];
    if (msg->range_min <= r && r <= msg->range_max) {
      // alpha in radians and always positive
      const float alpha = std::abs(i * msg->angle_increment + msg->angle_min);
      const float sin_alpha = std::sin(alpha);
      const float b = carWidth / 2.0;
      const float r_max = b / sin_alpha;
      if (r < r_max) {
        // distance to obstacle
        const float d = r * std::cos(alpha);
        if (d < d_min)
          d_min = d;
      }
    }
  }
  ROS_INFO("dist: %f", d_min);
}

void EmergencyBrake::timerCallback(const ros::TimerEvent &) {

  /*if (grid != nullptr) {

    geometry_msgs::PointStamped frontSensor;
    geometry_msgs::Vector3Stamped direction;
    tf::StampedTransform transform;
    try {
      auto now = ros::Time::now();
      tfListener.waitForTransform("/front_sensor", "/map", now,
                                  ros::Duration(DURATION / 2));
      tfListener.lookupTransform("/front_sensor", "/map", now, transform);

      geometry_msgs::PointStamped frontSensorIn;
      frontSensorIn.header.frame_id = "/front_sensor";
      tfListener.transformPoint("/map", frontSensorIn, frontSensor);

      geometry_msgs::Vector3Stamped directionIn;
      directionIn.vector.x = 1;
      directionIn.header.frame_id = "/front_sensor";
      tfListener.transformVector("/map", directionIn, direction);
    } catch (tf::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }
    // TODO remove
    nav_msgs::OccupancyGrid debugGrid;

    debugGrid.header.seq = gridId++;
    debugGrid.header.stamp = ros::Time::now();
    debugGrid.header.frame_id = grid->header.frame_id;

    debugGrid.info.height = 2 / grid->info.resolution;
    debugGrid.info.width = 2 / grid->info.resolution;
    debugGrid.info.resolution = grid->info.resolution;
    debugGrid.info.origin.orientation.w = 1;

    const uint32_t dataSize = debugGrid.info.width * debugGrid.info.height;
    for (uint32_t i = 0; i < dataSize; ++i)
      debugGrid.data.push_back(-1);

    debugGrid.info.origin.position = frontSensor.point;
    debugGrid.info.origin.position.x -= 1;
    debugGrid.info.origin.position.y -= 1;

    occupyCell(debugGrid, frontSensor.point);
    map_pub.publish(debugGrid);
}*/
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "emergency_brake");
  ros::NodeHandle n;
  EmergencyBrake emergencyBrake(&n);
  ROS_INFO("Emergency Brake launched");

  ros::spin();
  return 0;
}
