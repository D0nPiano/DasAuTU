#ifndef PS3_CONTROLLER_H
#define PS3_CONTROLLER_H

#include "autu_control/AutoController.h"
#include "ros/ros.h"
#include <sensor_msgs/Joy.h>

class PS3_Controller : public AutoController {
public:
  PS3_Controller(ros::NodeHandle *n, ros::Publisher *command_pub);
  ~PS3_Controller();
  void run();
  void handleJoystick(const sensor_msgs::Joy::ConstPtr &msg);

private:
  ros::NodeHandle *n;
  ros::Publisher *command_pub;
  ros::Subscriber joystick_pub;
  float r2, joystickLeftLeftwards;
};

#endif // PS3_CONTROLLER_H
