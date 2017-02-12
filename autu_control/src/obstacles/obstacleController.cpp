#ifndef _ObstacleController_CPP_
#define _ObstacleController_CPP_

#include "autu_control/obstacles/obstacleController.h"
#include "ros/ros.h"

Point point_new(float x, float y)
{
  Point a;
  a.x = x;
  a.y = y;
  return a;
}

ObstacleController::ObstacleController(ros::NodeHandle *n,
                                       ros::Publisher *command_pub,
                                       bool startDriving)
    : n(n), command_pub(command_pub) {
  ROS_INFO("New ObstacleController");

  plan_command_sub = n->subscribe<geometry_msgs::Twist>(
      "cmd_vel", 10, &ObstacleController::convertCommand, this);

  if(startDriving){
  	ROS_INFO("Warte auf Befehle...");
  } else {
  	ROS_INFO("Suche Rundkurs, loading from XML");
  	routeXML.LoadFile("/home/pses/route.xml");

  	// save data to array
  	tinyxml2::XMLNode * pRoot = routeXML.FirstChild();
  	if (pRoot == nullptr){
  		ROS_INFO("XML ist leer");
  		return;
  	}

  	tinyxml2::XMLElement * currentPointElement = pRoot->FirstChildElement("point");
	while (currentPointElement != nullptr)
		{
			float currentX;
			float currentY;
			currentPointElement->QueryFloatAttribute("x", &currentX);
			currentPointElement->QueryFloatAttribute("y", &currentY);

			Point currentPoint = point_new(currentX, currentY);

			points.push_back(currentPoint);
			currentPointElement = currentPointElement->NextSiblingElement("point");
		}

	for (Point i : points ) {
  		ROS_INFO("Points: [%f], [%f]", i.x, i.y);
	}

  }
}

ObstacleController::~ObstacleController() {
  ROS_INFO("Destroying ObstacleController");
  plan_command_sub.shutdown();
}

void ObstacleController::convertCommand(const geometry_msgs::Twist::ConstPtr& motionIn){
  command_data cmd;
  cmd.motor_level = int(motionIn->linear.x * 10);
  if(motionIn->linear.x > 0.0 && cmd.motor_level < 4){
  	cmd.motor_level = 4;
  } else if (motionIn->linear.x < 0.0 && cmd.motor_level > -10){
  	cmd.motor_level = -10;  	
  }
  
  cmd.steering_level = int((motionIn->angular.z) * 50);
  if(cmd.steering_level > 45){
  	cmd.steering_level = 45;
  } else if (cmd.steering_level < -45){
  	cmd.steering_level = -45;
  }

  if(cmd.steering_level > 4 && cmd.steering_level < 20){
    cmd.steering_level = 20;
  } else if (cmd.steering_level < -4 && cmd.steering_level > -20){
    cmd.steering_level = -20;
  }



  cmd.header.stamp = ros::Time::now();
  command_pub->publish(cmd);
  ros::spinOnce();
}

void ObstacleController::run() {

	// Trans
	try {
    tf::StampedTransform transform;
  		geometry_msgs::PointStamped currentPosition;

    	transformListener.waitForTransform("base_link", "/map", ros::Time(0),
                                         ros::Duration(0.1));
    	transformListener.lookupTransform("base_link", "/map", ros::Time(0),
                                        transform);

		geometry_msgs::PointStamped positionInBaseLink, startInBaseLaser;

		positionInBaseLink.point.x = 0;
		positionInBaseLink.point.y = 0;
		positionInBaseLink.header.frame_id = "/base_link";
		positionInBaseLink.header.stamp = ros::Time(0);
		transformListener.transformPoint("/map", positionInBaseLink, currentPosition);
		currentPosition.point.z = 0;

  		ROS_INFO("Current Position: [%f], [%f]", currentPosition.point.x, currentPosition.point.y);

    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }
}

#endif
