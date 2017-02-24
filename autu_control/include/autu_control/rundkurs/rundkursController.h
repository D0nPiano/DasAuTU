#ifndef _RundkursController_H_
#define _RundkursController_H_

/*
        If LaserScan is uninitialized, its range[0] is -1.0
        If SensorData is uninitialized, its range_sensor_left is -1.0
*/

#include "autu_control/AutoController.h"
#include "autu_control/rundkurs/curvedriver.h"
#include "autu_control/rundkurs/curvedriver2.h"
#include "autu_control/rundkurs/curvedriverconstant.h"
#include "autu_control/rundkurs/laser_utilities.h"
#include "autu_control/rundkurs/lowpass.h"
#include "autu_control/rundkurs/pidregler.h"

#include "std_msgs/String.h"
#include <exception>
#include <iostream>

#include "nav_msgs/Odometry.h"
#include <sensor_msgs/LaserScan.h>

#include "pses_basis/CarInfo.h"
#include "pses_basis/SensorData.h"
#include "ros/ros.h"
#include <pses_basis/Command.h>
typedef pses_basis::Command command_data;

class RundkursController : public AutoController {
public:
  RundkursController(ros::NodeHandle *n, ros::Publisher *commandPub);
  ~RundkursController();
  void run();

private:
  void stop();
  void getCurrentLaserScan(const sensor_msgs::LaserScan::ConstPtr &);
  void getCurrentSensorData(const pses_basis::SensorData::ConstPtr &);
  void odomCallback(const nav_msgs::OdometryConstPtr &msg);
  void carinfoCallback(const pses_basis::CarInfoConstPtr &msg);
  void simpleController();

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

  LaserUtil laserUtil;
  CurveDriverConstant curveDriver;
  Lowpass lowpass;
  PIDRegler pdController;

  bool initialized;
  uint8_t drivingState;
  int pdMaxMotorLevel;
  float curveRadius;
  double timeOfLastCorner;
  double afterCurveDeadtime;
};

#endif
