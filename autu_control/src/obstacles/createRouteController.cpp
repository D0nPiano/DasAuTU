#ifndef _CreateRouteController_CPP_
#define _CreateRouteController_CPP_

#include "ros/ros.h"

#include "autu_control/obstacles/createRouteController.h"
#include "std_msgs/Header.h"

CreateRouteController::CreateRouteController(ros::NodeHandle *n,
                                             ros::Publisher *command_pub)
    : n(n), command_pub(command_pub) {
  ROS_INFO("New CreateRouteController");

  // clicked_point_sub = n->subscribe("clicked_point", 10,
  // &CreateRouteController::addToPoints, this);

  poses_sub = n->subscribe("/move_base_simple/goal", 10,
                           &CreateRouteController::poseCallback, this);

  pRoot = routeXML.NewElement("route");
  routeXML.InsertFirstChild(pRoot);
}

CreateRouteController::~CreateRouteController() {
  ROS_INFO("Saving XML");

  routeXML.SaveFile("/home/pses/route.xml");

  ROS_INFO("Destroying CreateRouteController");

  plan_command_sub.shutdown();
}

void CreateRouteController::addToPoints(
    const geometry_msgs::PointStampedConstPtr &currentPoint) {
  float x = currentPoint->point.x;
  float y = currentPoint->point.y;
  ROS_INFO("Point coords: [%f], [%f]", x, y);

  tinyxml2::XMLElement *pElement = routeXML.NewElement("point");
  pElement->SetAttribute("x", x);
  pElement->SetAttribute("y", y);
  pRoot->InsertEndChild(pElement);
}

void CreateRouteController::poseCallback(
    const geometry_msgs::PoseStampedConstPtr &msg) {
  tinyxml2::XMLElement *pElement = routeXML.NewElement("point");
  pElement->SetAttribute("x", msg->pose.position.x);
  pElement->SetAttribute("y", msg->pose.position.y);
  pElement->SetAttribute("w", msg->pose.orientation.w);
  pElement->SetAttribute("z", msg->pose.orientation.z);
  pRoot->InsertEndChild(pElement);
}

void CreateRouteController::run() {}

#endif
