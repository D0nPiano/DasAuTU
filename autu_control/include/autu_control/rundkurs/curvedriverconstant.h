#ifndef CURVEDRIVERCONSTANT_H
#define CURVEDRIVERCONSTANT_H

#include "autu_control/rundkurs/laser_utilities.h"

#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "tf/transform_listener.h"

class CurveDriverConstant {
public:
  CurveDriverConstant(ros::NodeHandle &nh);

  /**
   * @brief Generates the commands to control the car in the curves.
   *
   * The commands consist of the motor-level and steering which are defined in
   * the launchfile.
   */
  void drive();

  /**
   * @brief Determines when the end of the curve is reached.
   *
   * At the beginning of the curve the car's orientation is saved in curveBegin.
   * This method compares the current yaw angle with the yaw angle from the
   * beginning.
   * On exceeding a given threshold (cornerEndAngle) the curve must have ended.
   *
   * @return True means the corner's end is reached otherwise false.
   */
  bool isAroundTheCorner() const;

  /**
   * @brief Prepares the curvedriver for a new curve. This method must be called
   * once after IsAtCurveBegin() started the curve-mode.
   *
   * curveBegin is set by this method.
   */
  void curveInit();

  /**
   * @brief Detects corners in the laserscan.
   *
   * To detect a curve firstly the laserscan is searched for steps between the
   * measured points.
   * If a possible corner is found it is verified that the step didn't come from
   * a glas pane.
   * A wall next to the corner is also required to reduce the false positives
   * produced by reflections on the ground.
   *
   * @param distanceToWall - The current distance to the left wall in m
   * @param speed - The current speed of the car in m/s
   * @return True means the car is near to a curve/corner.
   */
  bool isNextToCorner(float speed);

  /**
   * @brief Signals that the rollout-phase should begin now.
   * @return True -> Slow the car down
   */
  bool rolloutBegins() const;
  /**
   * @brief Signals that the curve-movement should be started.
   *
   * This means the distance between the car and the detected corner is lower
   * than precurveDistance.
   *
   * @return True -> Use drive() to send commands to the car for turning left.
   */
  bool isAtCurveBegin() const;

  /**
   * @brief Updates the laserscan.
   */
  void setLaserscan(const sensor_msgs::LaserScanConstPtr &scan);

  /**
   * @brief Updates odometry data.
   */
  void setOdom(const nav_msgs::OdometryConstPtr &msg);

private:
  /**
   * @brief Updates the scanOffset.
   *
   * To calculate the scanOffset
   *
   * @param speed - The current speed of the car in m/s
   */
  void updateScanOffset(float speed);

  /**
   * @brief Determines if the detected corner was only produced by a glass pane
   * or a reflection on the wall.
   * @param cornerX - x-coordinate of the detected corner in base_laser
   * @param cornerY - y-coordinate of the detected corner in base_laser
   * @return True = glass pane or reflection produced the corner
   */
  bool isNextToGlas(float cornerX, float cornerY);

  /**
   * @brief Looks for a wall in the environment of the detected corner.
   * @param cornerX - x-coordinate of the detected corner in base_laser
   * @param cornerY - y-coordinate of the detected corner in base_laser
   * @return True = wall was found near the corner
   */
  bool wallFound(float cornerX, float cornerY);

  /**
   * @brief Calculates the distance between the detected corner and the current
   * position of the car.
   * @return Distance to the detected corner in m
   */
  float getCurrentDistanceToCorner() const;

  ros::Publisher commandPub;
  ros::Publisher cornerPub;
  ros::Publisher infoPub;

  tf::TransformListener transformListener;
  tf::StampedTransform transform;
  sensor_msgs::LaserScanConstPtr laserscan;
  nav_msgs::OdometryConstPtr odom;

  /**
   * @brief Car's position when the curve was detected
   */
  geometry_msgs::Pose cornerSeen;

  /**
   * @brief Car's position when entering the curve.
   */
  geometry_msgs::Pose curveBegin;

  /**
   * @brief Position of the detected corner in frame odom
   */
  geometry_msgs::PointStamped cornerInOdom;

  /**
   * @brief Steering level in the curve
   */
  int steering;

  /**
   * @brief Motor level in the curve
   */
  int maxMotorLevel;

  /**
   * @brief The step between two points in the laserscan must exceed this value
   * to be considered a possible corner.
   */
  float cornerThreshold;

  /**
   * @brief The distance the car has to travel with a turned off motor to slow
   * down enough for the curve
   */
  float rolloutDistance;

  /**
   * @brief Additional offset before a corner
   *
   * A corner can only be seen from a certain distance. If the distance gets too
   * low the corner will disappear from the scan.
   * So the corner detection is performed earlier and the corner's position
   * memorized.
   */
  float blindnessOffset;

  /**
   * @brief Offset to correct the laserscan delay in m
   *
   * The generation of the laserscans takes a lot of time. While this generation
   * the car moves forward.
   * This offset was introduced to compensate the deviation.
   */
  float scanOffset;

  /**
   * @brief Timestamp when scanOffset was updated the last time
   */
  double scanOffsetStamp;

  /**
   * @brief Threshold used by isAroundCorner() to determine if the curve has
   * ended.
   */
  float cornerEndAngle;

  /**
   * @brief If the distance to the corner is smaller than precurveDistance the
   * car starts to steer left.
   */
  float precurveDistance;

  /**
   * @brief The coordinates of the detected corner (frame: base_laser).
   *
   * The scanOffset was already considered.
   */
  struct Point {
    float x;
    float y;
  } corner;
};

#endif // CURVEDRIVERCONSTANT_H
