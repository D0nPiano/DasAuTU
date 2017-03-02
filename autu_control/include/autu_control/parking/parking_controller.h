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
#include <vector>

/**
 * @brief The ParkingController drives the car in a parking spot.
 *
 * At first the controller searches for a corner in the current laserscan.
 * It is assumed that the found corner is the rear left end of a parked car.
 * After that the controller tries to park behind that car by driving to the
 * start-position.
 * Therefor a pd-controller is used.
 * On reaching the start-position the car starts to drive backwards while
 * steering right till the yaw angle is equal to theta.
 * If this happens the steering changes from right to left.
 * Once again the yaw angle is used to determine when to stop.
 *
 * The variables are named after their equivalents in the scientific paper about
 * parking in the doc-folder.
 */

class ParkingController : public AutoController {
public:
  ParkingController(ros::NodeHandle &nh);
  /**
   * @brief Saves the current odometry.
   */
  void odomCallback(const nav_msgs::OdometryConstPtr &msg);
  /**
   * @brief Saves the current laserscan.
   */
  void laserscanCallback(const sensor_msgs::LaserScanConstPtr &msg);
  void run();

private:
  /**
   * @brief Calculates the trajectory for visualisation purposes.
   *
   * Generates a trajectory for the right rear wheel. The trajectory is
   * represented by vectors pointing from the corner.
   *
   * @return Trajectory consisting of vectors
   */
  std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>
  calcTrajectory();
  /**
   * @brief Publishes the calculated trajectory.
   *
   * Publishes the trajectory for the left and for the right rear wheel for the
   * backwards movement.
   */
  void publishParkingTrajectory();

  // ROS Publishers and Subscribers
  ros::Publisher command_pub;
  ros::Subscriber odom_sub;
  ros::Subscriber laser_sub;
  nav_msgs::OdometryConstPtr odom;
  sensor_msgs::LaserScanConstPtr laserscan;

  geometry_msgs::Pose curveBegin; // is used to store the car's orientation on
                                  // the begin of the turns
  geometry_msgs::PointStamped corner; // corner is in frame odom
  geometry_msgs::PointStamped start; // start is in frame odom
  
  tf::TransformListener transformListener;
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
