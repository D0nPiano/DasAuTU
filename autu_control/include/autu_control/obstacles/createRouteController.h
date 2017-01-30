#ifndef _CreateRouteController_H_
#define _CreateRouteController_H_

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

#include "ros/ros.h"

#include "nav_msgs/Odometry.h"
#include <sensor_msgs/LaserScan.h>

#include "pses_basis/CarInfo.h"
#include "pses_basis/SensorData.h"
#include <pses_basis/Command.h>


#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>

typedef pses_basis::Command command_data;

class CreateRouteController : public AutoController {
public:
  CreateRouteController(ros::NodeHandle *n, ros::Publisher *command_pub);
  ~CreateRouteController();
  void run();
private:
	ros::NodeHandle* n;
	void addToPoints(const geometry_msgs::PointStampedConstPtr& currentPoint);
	ros::Subscriber plan_command_sub;
	ros::Subscriber clicked_point_sub;
	ros::Publisher* command_pub;
	tinyxml2::XMLDocument routeXML;
	tinyxml2::XMLNode * pRoot;
};

#endif
