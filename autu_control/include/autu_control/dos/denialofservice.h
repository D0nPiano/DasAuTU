#ifndef DENIALOFSERVICE_H
#define DENIALOFSERVICE_H

#include "autu_control/AutoController.h"
#include "pses_basis/Command.h"
#include "ros/ros.h"

class DenialOfService : public AutoController {
public:
  DenialOfService(ros::NodeHandle &n);
  ~DenialOfService() {}
  void run();

private:
  void sendMotorCommand(const ros::TimerEvent &);
  ros::Timer timer;
  ros::Publisher cmd_pub;
  int i;
};

#endif // DENIALOFSERVICE_H
