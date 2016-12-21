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
  ros::Subscriber joystick_sub;
  // gaspedal range is from -1.0 (max backward) to 1.0 (max forward)
  float gaspedal;
  // joystickLeftLeftwards range is from -1.0 (right) to 1.0 (left)
  float joystickLeftLeftwards;
  int max_motorlevel, max_steering;
  // the timestamp from the last message received from the gamepad in seconds
  double lastUpdateFromGamepad;
};

#endif // PS3_CONTROLLER_H
