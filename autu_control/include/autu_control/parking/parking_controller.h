#ifndef PARKINGCONTROLLER_H
#define PARKINGCONTROLLER_H

#include "Eigen/Dense"
#include "autu_control/AutoController.h"
#include "autu_control/rundkurs/laser_utilities.h"
#include "autu_control/rundkurs/pidregler.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <tf/transform_broadcaster.h>
#include <vector>

class ParkingController : public AutoController {
public:
  ParkingController(ros::NodeHandle &nh);
  void odomCallback(const nav_msgs::OdometryConstPtr &msg);
  void laserscanCallback(const sensor_msgs::LaserScanConstPtr &msg);
  void run();

private:
  std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>
  calcTrajectory();
  void publishParkingTrajectory();
  ros::Publisher command_pub;
  ros::Subscriber odom_sub;
  ros::Subscriber laser_sub;
  nav_msgs::OdometryConstPtr odom;
  sensor_msgs::LaserScanConstPtr laserscan;
  geometry_msgs::Pose curveBegin;
  // start is in frame parking
  geometry_msgs::PointStamped start;
  tf::TransformListener transformListener;
  tf::TransformBroadcaster transformBroadcaster;
  tf2_ros::StaticTransformBroadcaster staticTFBroadcaster;
  LaserUtil laserUtil;
  PIDRegler pidRegler;
  uint8_t state;
  int16_t velocity_forward, velocity_backward;
  int16_t maxSteeringLeft, maxSteeringRight;
  float theta;
  float regulator_d, regulator_p;
  float correction_x;
  float safety_distance;
  float b, w;
  float r;
  float r_e;
  float alpha, beta;
  float delta;
#ifndef NDEBUG
  ros::Publisher trajectory_pub;
  ros::Publisher trajectory_odom_pub;
#endif
};

#endif // PARKINGCONTROLLER_H
