#ifndef _ObstacleController_CPP_
#define _ObstacleController_CPP_

#include "autu_control/obstacles/obstacleController.h"
#include "ros/ros.h"

Point point_new(float x, float y) {
  Point a;
  a.x = x;
  a.y = y;
  return a;
}

ObstacleController::ObstacleController(ros::NodeHandle *n,
                                       ros::Publisher *command_pub,
                                       bool startDriving)
    : n(n), command_pub(command_pub), us_front(0) {
  ROS_INFO("New ObstacleController");

  goal_pub =
      n->advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1000);

  plan_command_sub = n->subscribe<geometry_msgs::Twist>(
      "cmd_vel", 10, &ObstacleController::convertCommand, this);

  sensorDataSub = n->subscribe<pses_basis::SensorData>(
      "pses_basis/sensor_data", 10, &ObstacleController::sensorDataCallback,
      this);
  //['/front_us_range', '/left_us_range', '/right_us_range']
  usFrontPub = n->advertise<sensor_msgs::Range>("/front_us_range", 10);
  usLeftPub = n->advertise<sensor_msgs::Range>("/left_us_range", 10);
  usRightPub = n->advertise<sensor_msgs::Range>("/right_us_range", 10);

  if (!startDriving) {
    ROS_INFO("Warte auf Befehle...");
  } else {
    ROS_INFO("Suche Rundkurs, loading from XML");
    routeXML.LoadFile("/home/pses/route.xml");

    // save data to array
    tinyxml2::XMLNode *pRoot = routeXML.FirstChild();
    if (pRoot == nullptr) {
      ROS_INFO("XML ist leer");
      return;
    }

    tinyxml2::XMLElement *currentPointElement =
        pRoot->FirstChildElement("point");
    while (currentPointElement != nullptr) {
      float currentX;
      float currentY, w, z;
      currentPointElement->QueryFloatAttribute("x", &currentX);
      currentPointElement->QueryFloatAttribute("y", &currentY);
      currentPointElement->QueryFloatAttribute("w", &w);
      currentPointElement->QueryFloatAttribute("z", &z);

      geometry_msgs::Pose currentPoint;
      currentPoint.position.x = currentX;
      currentPoint.position.y = currentY;
      currentPoint.orientation.w = w;
      currentPoint.orientation.z = z;

      poses.push_back(currentPoint);
      currentPointElement = currentPointElement->NextSiblingElement("point");
    }

    for (Point i : points) {
      ROS_INFO("Points: [%f], [%f]", i.x, i.y);
    }
  }

  currentGoal = -1;
}

ObstacleController::~ObstacleController() {
  ROS_INFO("Destroying ObstacleController");
  plan_command_sub.shutdown();
}

void ObstacleController::sendNextGoal() {
  ROS_INFO("Sending Next Goal");
  currentGoal++;
  if (currentGoal == (int)poses.size()) {
    currentGoal = 0;
  }
  geometry_msgs::PoseStamped nextGoal;
  /*nextGoal.pose.position.x = points[currentGoal].x;
  nextGoal.pose.position.y = points[currentGoal].y;
  nextGoal.pose.position.z = 0.0;

  nextGoal.pose.orientation.x = 0.0;
  nextGoal.pose.orientation.y = 0.0;
  nextGoal.pose.orientation.z = 0.015;
  nextGoal.pose.orientation.w = 0.998;*/

  nextGoal.pose = poses[currentGoal];

  nextGoal.header.stamp = ros::Time::now();
  nextGoal.header.frame_id = "map";
  goal_pub.publish(nextGoal);
  ros::spinOnce();
}

bool ObstacleController::isNearToNextGoal(
    const geometry_msgs::PointStamped *currentPosition) {
  if (currentGoal == -1) {
    return true;
  }

  /*ROS_INFO("Current Position: [%f], [%f]", currentPosition->point.x,
  currentPosition->point.y);
  ROS_INFO("currentGoal: [%d]", currentGoal);
  ROS_INFO("Vektor X: [%f]", points[currentGoal].x);
  */

  float diffx =
      pow(currentPosition->point.x - poses[currentGoal].position.x, 2);
  float diffy =
      pow(currentPosition->point.y - poses[currentGoal].position.y, 2);
  float distance = sqrt(diffx + diffy);

  // ROS_INFO("Distance: [%f]", distance);
  return (distance < 1.0);
}

void ObstacleController::convertCommand(
    const geometry_msgs::Twist::ConstPtr &motionIn) {
  command_data cmd;

  const float alpha_max = M_PI_2 - std::atan(0.85 / 0.28);
  const float maxSteeringLeft = 45;
  const float maxSteeringRight = -45;
  const float steeringWidth = maxSteeringLeft - maxSteeringRight;
  const float steeringMitte = maxSteeringRight + steeringWidth / 2;

  cmd.steering_level =
      steeringMitte + (motionIn->angular.z / alpha_max) * steeringWidth / 2;

  if (cmd.steering_level > 45) {
    cmd.steering_level = 45;
  } else if (cmd.steering_level < -45) {
    cmd.steering_level = -45;
  }

  /*if (us_front > 0.65 && std::abs(cmd.steering_level - steeringMitte) < 10) {
    cmd.motor_level = motionIn->linear.x * 50;
  } else*/
  cmd.motor_level = int(motionIn->linear.x * 20);

  if (motionIn->linear.x > 0 && cmd.motor_level < 5) {
    cmd.motor_level = 5;
  } else if (motionIn->linear.x < 0.0 && cmd.motor_level > -10) {
    cmd.motor_level = -10;
  }

  ROS_INFO("motionIn->linear.x: %f -> Motor Level: %i us_front: %f steering "
           "level: %f",
           motionIn->linear.x, cmd.motor_level, us_front,
           std::abs(cmd.steering_level - steeringMitte));
  if (cmd.motor_level > 13)
    cmd.motor_level = 13;
  else if (cmd.motor_level < -12)
    cmd.motor_level = -12;

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
    transformListener.transformPoint("/map", positionInBaseLink,
                                     currentPosition);
    currentPosition.point.z = 0;

    if (isNearToNextGoal(&currentPosition)) {
      sendNextGoal();
    }

  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }
}

void ObstacleController::sensorDataCallback(
    const pses_basis::SensorDataConstPtr &msg) {
  us_front = msg->range_sensor_front;
  sensor_msgs::Range range;
  range.min_range = 0.02;
  range.max_range = 0.5;
  range.field_of_view = 0.5;
  range.range = msg->range_sensor_front;
  range.radiation_type = 0;
  usFrontPub.publish(range);
  range.range = msg->range_sensor_left;
  // usLeftPub.publish(range);
  range.range = msg->range_sensor_right;
  // usRightPub.publish(range);
}

#endif
