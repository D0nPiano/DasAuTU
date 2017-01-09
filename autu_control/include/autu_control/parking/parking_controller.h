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

class ParkingController : public AutoController {
public:
  ParkingController(ros::NodeHandle &nh);
  void odomCallback(const nav_msgs::OdometryConstPtr &msg);
  void laserscanCallback(const sensor_msgs::LaserScanConstPtr &msg);
  void run();

private:
  void publishParkingTrajectory();
  ros::Publisher command_pub;
  ros::Subscriber odom_sub;
  ros::Subscriber laser_sub;
  nav_msgs::OdometryConstPtr odom;
  sensor_msgs::LaserScanConstPtr laserscan;
  geometry_msgs::Pose curveBegin;
  // corner is in frame odom
  geometry_msgs::PointStamped corner;
  // start is in frame odom
  geometry_msgs::PointStamped start;
  tf::TransformListener transformListener;
  LaserUtil laserUtil;
  PIDRegler pidRegler;
  uint8_t state;
  int16_t velocity_forward, velocity_backward;
  int16_t maxSteering;
  float theta;
  float regulator_d, regulator_p;
  float a, b, w;
  float r;
  float r_e;
  float alpha, beta;
  float delta;
#ifndef NDEBUG
  ros::Publisher trajectory_pub;
#endif
};

#endif // PARKINGCONTROLLER_H
