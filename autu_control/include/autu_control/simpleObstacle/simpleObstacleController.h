#ifndef _SimpleObstacleController_H_
#define _SimpleObstacleController_H_

#define PI 3.14159265

/*
        If LaserScan is uninitialized, its range[0] is -1.0
        If SensorData is uninitialized, its range_sensor_left is -1.0
*/

#include "autu_control/AutoController.h"

#include "autu_control/rundkurs/lowpass.h"
#include "autu_control/rundkurs/pidregler.h"
#include "autu_control/rundkurs/curvedriverconstant.h"
#include "std_msgs/String.h"
#include <exception>
#include <iostream>
#include <memory>

#include "nav_msgs/Odometry.h"
#include <sensor_msgs/LaserScan.h>

#include "pses_basis/CarInfo.h"
#include "pses_basis/SensorData.h"
#include "ros/ros.h"
#include <pses_basis/Command.h>
typedef pses_basis::Command command_data;

class SimpleObstacleController : public AutoController {
public:
  SimpleObstacleController(ros::NodeHandle *n, ros::Publisher *command_pub);
  ~SimpleObstacleController();
  void run();
private:
  void getCurrentLaserScan(const sensor_msgs::LaserScan::ConstPtr &);
  void getCurrentSensorData(const pses_basis::SensorData::ConstPtr &);
  void updateDistanceToObstacle();
  void getBestHeadingAngle();
  void simpleController();
  int getBestSpeed();
  int getBestSteering();
  float getWrackingDistance();
  void odomCallback(const nav_msgs::OdometryConstPtr &msg);
  void carinfoCallback(const pses_basis::CarInfoConstPtr &msg);
  ros::NodeHandle *n;
  ros::Publisher *command_pub;
  ros::Subscriber laser_sub;
  ros::Subscriber sensor_sub;
  ros::Subscriber odomSub;
  ros::Subscriber carinfoSub;
  sensor_msgs::LaserScanConstPtr currentLaserScan;
  pses_basis::SensorDataConstPtr currentSensorData;
  pses_basis::CarInfoConstPtr currentCarInfo;
  nav_msgs::OdometryConstPtr odomData;
  bool initialized;
  float obstacleDistace;
  float currentHeadingAngle;
  PIDRegler* pidRegler;
  CurveDriverConstant* curveDriverConstant;
  float minWallDist;
  float steeringMulti;
  float distortPow;
  float distortUSInfluencePow;
  float obstacleSteeringPow;
  int PIDMotorLevel;
  float PIDWallDistance;
  float obstacleMotorLevel;
  float PIDP;
  float PIDD;
  Lowpass lowpass;
  bool curveAhead;
  double curveTimer;
  bool curveBegin;
};

#endif
