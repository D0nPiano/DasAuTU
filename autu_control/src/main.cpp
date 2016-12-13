#include "autu_control/rundkursController.h"
#include "autu_control/remoteController.h"
#include "autu_control/AutoController.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>

#define RUNTIMER_DELTA .1

void getMode(const std_msgs::String::ConstPtr &msg, std::string *mode, bool *modeChanged) {
  *mode = msg->data;
  *modeChanged = true;
}

void runTimerCallback(const ros::TimerEvent&, AutoController **rndCtrl, std::string *mode, bool *modeChanged, ros::NodeHandle *n){
  //ROS_INFO("called");
  if(*modeChanged){
    ROS_INFO("Mode CHANGED");
    *modeChanged = false;

    delete(*rndCtrl);
    if(!mode->compare("Follow Wall")){
      *rndCtrl = new RundkursController(n);
    } else {
      *rndCtrl = new RemoteController(n);
    }
  }

  if(mode->compare("Remote Control")){
      (*rndCtrl)->run();
  }
}

int main(int argc, char **argv) {

  std::string mode("Remote Control");
  bool modeChanged;
  ros::init(argc, argv, "main");
  ros::NodeHandle n;
  AutoController *rndCtrl = new RemoteController(&n); // TODO: Remove Eventually


  ros::Timer runTimer = n.createTimer(ros::Duration(RUNTIMER_DELTA),
    std::bind(runTimerCallback, std::placeholders::_1, &rndCtrl, &mode, &modeChanged, &n));

  ros::Subscriber submode = n.subscribe<std_msgs::String>(
      "pses_basis/mode_control", 1000,
      std::bind(getMode, std::placeholders::_1, &mode, &modeChanged));

  ros::spin();
  return 0;
}