#ifndef _RundkursController_H_
#define _RundkursController_H_

#include "autu_control/AutoController.h"
#include "autu_control/rundkurs/curvedriverconstant.h"
#include "autu_control/rundkurs/lowpass.h"
#include "autu_control/rundkurs/pdcontroller.h"

#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"

#include "pses_basis/CarInfo.h"
#include "pses_basis/Command.h"
#include "pses_basis/SensorData.h"

/**
 * @brief The controller for the circuit without obstacles
 *
 * From the controllers point of view the circuit contains only straight
 * sections and curves.
 * While on the straight sections of the track a pd-controller maintains the
 * distance to the left wall via the left ultrasonic sensor.
 * The curves at the end of the straight sections are detected with the kinect,
 * more precisely with the generated laserscans from depthimage_to_laserscan
 * node.
 * To drive through a curve a constant steering level is used.
 * The curve's end is determined by comparing the current car's yaw angle to the
 * yaw angle at the beginning of the curve.
 * By exceeding a given threshold the curve-mode finishes.
 *
 * A minimum time span between two curves is also defined to prevent the car
 * from recognizing a second curve at the end of another curve, which is
 * obviously not existing.
 */
class RundkursController : public AutoController {
public:
  RundkursController(ros::NodeHandle *n, ros::Publisher *commandPub);
  ~RundkursController();
  void run();

private:
  /**
   * @brief Stops the car immediatly.
   *
   * Sends a motor-command with motorlevel = 0 and steering = 0 to stop the car.
   */
  void stop();

  /**
   * @brief Updates the laserscan.
   */
  void getCurrentLaserScan(const sensor_msgs::LaserScan::ConstPtr &);

  /**
   * @brief Updates values from sensors.
   *
   * Contains the ultrasonic values.
   */
  void getCurrentSensorData(const pses_basis::SensorData::ConstPtr &);

  /**
   * @brief Updates the odometry data.
   */
  void odomCallback(const nav_msgs::OdometryConstPtr &msg);

  /**
   * @brief Updates car information
   *
   * Contains the car's current speed.
   */
  void carinfoCallback(const pses_basis::CarInfoConstPtr &msg);

  /**
   * @brief Drives the car through the circuit.
   */
  void controlCar();

  ros::NodeHandle *n;
  ros::Publisher *commandPub;
  ros::Subscriber laserscanSub;
  ros::Subscriber sensorDataSub;
  ros::Subscriber odomSub;
  ros::Subscriber carinfoSub;

  sensor_msgs::LaserScanConstPtr currentLaserScan;
  pses_basis::SensorDataConstPtr currentSensorData;
  pses_basis::CarInfoConstPtr currentCarInfo;
  nav_msgs::OdometryConstPtr odomData;

  CurveDriverConstant curveDriver;
  Lowpass lowpass;
  PDController pdController;

  /**  @brief If false the sensors aren't ready yet. */
  bool initialized;

  /** @brief The current state of the intern FSM */
  uint8_t drivingState;

  /** @brief The motorlevel for the pd-controller */
  int pdMaxMotorLevel;

  /**
   * @brief Timestamp of the last corner the car drove.
   *
   * The timestamp refers to the curve's end.
   */
  double timeOfLastCorner;

  /**
   * @brief Minimum time span between two curves
   */
  double afterCurveDeadtime;
};

#endif
