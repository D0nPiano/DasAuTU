#include "autu_control/rundkursController.h"
#include "autu_control/remoteController.h"
#include "autu_control/AutoController.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include "pses_basis/SensorData.h"
#include <pses_basis/Command.h>
typedef pses_basis::Command command_data;

#define RUNTIMER_DELTA .01

void getMode(const std_msgs::String::ConstPtr &msg, std::string *mode, bool *modeChanged) {
  *mode = msg->data;
  *modeChanged = true;
}

void getFrontRange(const pses_basis::SensorData::ConstPtr&  msg, float * frontRange){
  *frontRange = msg->range_sensor_front;
}


void runTimerCallback(const ros::TimerEvent&, AutoController **rndCtrl, std::string *mode, bool *modeChanged,
          ros::NodeHandle *n, float * frontRange, ros::Publisher *command_pub){

  // ----- Mode Changer -----
  if(*modeChanged){
    ROS_INFO("Mode CHANGED");
    *modeChanged = false;

    delete(*rndCtrl);
    if(!mode->compare("Follow Wall")){
      *rndCtrl = new RundkursController(n, command_pub);
    } else {
      *rndCtrl = new RemoteController(n, command_pub);
    }
  }

  // ---- Notbremse --------
  if(*frontRange < 0.2){
    ROS_INFO("NOTBREMSE");
    command_data cmd;
    cmd.motor_level = -5;
    cmd.steering_level = 0;
    cmd.header.stamp = ros::Time::now();
    command_pub->publish(cmd);
    ros::spinOnce();
    return;
  }

  // ----- Call to Controller -----
  if(mode->compare("Remote Control")){
      (*rndCtrl)->run();
  }
}

int main(int argc, char **argv) {

  std::string mode("Remote Control");
  bool modeChanged;
  ros::init(argc, argv, "main");
  ros::NodeHandle n;
  float frontRange;

  ros::Publisher command_pub = n.advertise<command_data>("pses_basis/command", 1000);

  AutoController *rndCtrl = new RemoteController(&n, &command_pub);


  ros::Timer runTimer = n.createTimer(ros::Duration(RUNTIMER_DELTA),
    std::bind(runTimerCallback, std::placeholders::_1, &rndCtrl, &mode, &modeChanged, &n, &frontRange, &command_pub));

  ros::Subscriber submode = n.subscribe<std_msgs::String>(
      "pses_basis/mode_control", 1000,
      std::bind(getMode, std::placeholders::_1, &mode, &modeChanged));

  ros::Subscriber subSensors = n.subscribe<pses_basis::SensorData>(
      "pses_basis/sensor_data", 1000,
      std::bind(getFrontRange, std::placeholders::_1, &frontRange));

  ros::spin();
  return 0;
}