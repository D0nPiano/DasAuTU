#include "autu_control/AutoController.h"
#include "autu_control/gamepad/ps3_controller.h"
#include "autu_control/parking/parking_controller.h"
#include "autu_control/remoteController.h"
#include "autu_control/rundkurs/rundkursController.h"
#include "autu_control/obstacles/obstacleController.h"
#include "autu_control/obstacles/createRouteController.h"

#include "pses_basis/SensorData.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <pses_basis/Command.h>
typedef pses_basis::Command command_data;

#define RUNTIMER_DELTA .03

void getMode(const std_msgs::String::ConstPtr &msg, std::string *mode,
             bool *modeChanged) {
  *mode = msg->data;
  *modeChanged = true;
}

void runTimerCallback(const ros::TimerEvent &, AutoController **rndCtrl,
                      std::string *mode, bool *modeChanged, ros::NodeHandle *n,
                      ros::Publisher *command_pub) {

  // ----- Mode Changer -----
  if (*modeChanged) {
    ROS_INFO("Mode CHANGED");
    *modeChanged = false;

    delete (*rndCtrl);
    if (!mode->compare("Follow Wall")) {
      *rndCtrl = new RundkursController(n, command_pub);
    } else if (!mode->compare("Gamepad")) {
      *rndCtrl = new PS3_Controller(n, command_pub);
    } else if (!mode->compare("Park Car")) {
      *rndCtrl = new ParkingController(*n);
    } else if (!mode->compare("Trajectories")) {
      *rndCtrl = new ObstacleController(n, command_pub, false);
    } else if (!mode->compare("Roundtrip w. Obstacles")) {
      *rndCtrl = new ObstacleController(n, command_pub, true);
    } else if (!mode->compare("Create Route")) {
      *rndCtrl = new CreateRouteController(n, command_pub);
    } else {
      *rndCtrl = new RemoteController(n, command_pub);
    }
  }

  // ----- Call to Controller -----
  (*rndCtrl)->run();
}

int main(int argc, char **argv) {

  std::string mode("Remote Control");
  bool modeChanged;
  ros::init(argc, argv, "main");
  ros::NodeHandle n;

  ros::Publisher command_pub = n.advertise<command_data>("autu/command", 1000);

  AutoController *rndCtrl = new RemoteController(&n, &command_pub);

  ros::Timer runTimer =
      n.createTimer(ros::Duration(RUNTIMER_DELTA),
                    std::bind(runTimerCallback, std::placeholders::_1, &rndCtrl,
                              &mode, &modeChanged, &n, &command_pub));

  ros::Subscriber submode = n.subscribe<std_msgs::String>(
      "pses_basis/mode_control", 1000,
      std::bind(getMode, std::placeholders::_1, &mode, &modeChanged));

  ros::spin();
  return 0;
}
