#ifndef _ObstacleController_H_
#define _ObstacleController_H_

#define PI 3.14159265

/*
        If LaserScan is uninitialized, its range[0] is -1.0
        If SensorData is uninitialized, its range_sensor_left is -1.0
*/

#include "autu_control/AutoController.h"
#include "autu_control/obstacles/tinyxml2/tinyxml2.h"

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

#include "tf/tf.h"
#include "tf/transform_listener.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"

typedef pses_basis::Command command_data;

typedef struct { float x, y; } Point;

class ObstacleController : public AutoController {
public:
  ObstacleController(ros::NodeHandle *n, ros::Publisher *command_pub,
                     bool startDriving);
  ~ObstacleController();
  void run();

private:
  ros::NodeHandle *n;
  void convertCommand(const geometry_msgs::Twist::ConstPtr &motionIn);
  void sendNextGoal();
  bool isNearToNextGoal(const geometry_msgs::PointStamped *currentPosition);
  void generateNextGoal(const ros::TimerEvent &);
  ros::Timer timer;
  ros::Subscriber plan_command_sub;
  ros::Publisher *command_pub;
  ros::Publisher goal_pub;
  tinyxml2::XMLDocument routeXML;
  std::vector<Point> points;
  tf::TransformListener transformListener;
  int currentGoal;
};

#endif
