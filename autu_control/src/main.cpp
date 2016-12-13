/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the
 * names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
#include "autu_control/rundkursController.h"
#include "autu_control/AutoController.h"

#include "pses_basis/SensorData.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <pses_basis/Command.h>
#include <math.h>

typedef pses_basis::Command command_data;
#include <sensor_msgs/LaserScan.h>

#define CORNER_SENSITIVITY 1.0
#define RUNTIMER_DELTA .1
#define CURVETIMER_DELTA .1
#define PI 3.14159265

#define RANGE_START 349
#define RANGE_DIFF 10
#define ANGLE_OFFSET 0.0

//#define RANGE_START 349
//#define RANGE_DIFF 10

void getMode(const std_msgs::String::ConstPtr &msg, std::string *mode, bool *modeChanged) {
  *mode = msg->data;
  *modeChanged = true;
}

void runTimerCallback(const ros::TimerEvent&, AutoController *rndCtrl, std::string *mode, bool *modeChanged, ros::NodeHandle *n){
  if(*modeChanged){
    ROS_INFO("CHANGED");
    *modeChanged = false;

    if(!mode->compare("Follow Wall")){
      ROS_INFO("I will follow you");
      rndCtrl = new RundkursController(n);
    }
  }
  if(mode->compare("Remote Control")){
      rndCtrl->run();
  }
}

int main(int argc, char **argv) {

  std::string mode("Remote Control"); // false falls es nicht Startzustand
  bool modeChanged;
  ros::init(argc, argv, "main");
  ros::NodeHandle n;
  AutoController *rndCtrl = new RundkursController(&n); // TODO: Remove Eventually


  ros::Timer runTimer = n.createTimer(ros::Duration(RUNTIMER_DELTA),
    std::bind(runTimerCallback, std::placeholders::_1, rndCtrl, &mode, &modeChanged, &n));

  ros::Subscriber submode = n.subscribe<std_msgs::String>(
      "pses_basis/mode_control", 1000,
      std::bind(getMode, std::placeholders::_1, &mode, &modeChanged));

  ros::spin();
  return 0;
}
// %EndTag(FULLTEXT)%
