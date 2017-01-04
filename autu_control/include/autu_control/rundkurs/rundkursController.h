#ifndef _RundkursController_H_
#define _RundkursController_H_

#define PI 3.14159265

/*
        If LaserScan is uninitialized, its range[0] is -1.0
        If SensorData is uninitialized, its range_sensor_left is -1.0
*/

#include "autu_control/AutoController.h"
#include "autu_control/rundkurs/curvedriver.h"
#include "autu_control/rundkurs/curvedriver2.h"
#include "autu_control/rundkurs/laserDetector.h"
#include "autu_control/rundkurs/laser_utilities.h"
#include "autu_control/rundkurs/lowpass.h"
#include "autu_control/rundkurs/pidregler.h"

#include "std_msgs/String.h"
#include <exception>
#include <iostream>
#include <memory>

#include "nav_msgs/Odometry.h"
#include <sensor_msgs/LaserScan.h>

#include "pses_basis/SensorData.h"
#include "ros/ros.h"
#include <pses_basis/Command.h>
typedef pses_basis::Command command_data;

class RundkursController : public AutoController {
public:
  RundkursController(ros::NodeHandle *n, ros::Publisher *command_pub);
  ~RundkursController();
  void run();

private:
  void stop();
  void getCurrentLaserScan(const sensor_msgs::LaserScan::ConstPtr &);
  void getCurrentSensorData(const pses_basis::SensorData::ConstPtr &);
  void odomCallback(const nav_msgs::OdometryConstPtr &msg);
  float getDistanceToWall();
  void simpleController();
  ros::NodeHandle *n;
  ros::Publisher *command_pub;
  ros::Subscriber laser_sub;
  ros::Subscriber sensor_sub;
  ros::Subscriber odom_sub;
  LaserUtil laserUtil;
  Lowpass lowpass;
  sensor_msgs::LaserScanConstPtr currentLaserScan;
  pses_basis::SensorDataConstPtr currentSensorData;
  std::unique_ptr<LaserDetector> laserDetector;
  bool initialized;
  uint8_t drivingState;
  PIDRegler pidRegler;
  CurveDriver2 curveDriver;
  nav_msgs::OdometryConstPtr odomData;

#ifndef NDEBUG
  ros::Publisher us_raw_dbg_pub, us_lp_dbg_pub, ransac_dbg_pub;
#endif
};

#endif
