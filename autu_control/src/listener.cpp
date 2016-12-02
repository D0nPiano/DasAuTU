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
#include "autu_control/listener.h"
#include "pses_basis/SensorData.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <pses_basis/Command.h>
typedef pses_basis::Command command_data;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
// %Tag(CALLBACK)%
void chatterCallback(const pses_basis::SensorData::ConstPtr &msg,
                     ros::Publisher &chatter_pub, float *currentVelPtr,
                     bool *mode) {
  if (*mode == true) {
    float currentRange = msg->range_sensor_front;
    ROS_INFO("distance to front: [%f]", msg->range_sensor_front);
    ROS_INFO("currentVel: [%f]", *currentVelPtr);

    if (currentRange < 1.5 && *currentVelPtr > 0) {
      command_data cmd;
      cmd.header.stamp = ros::Time::now();
      cmd.motor_level = 0;

      // ROS_INFO("%s", retmsg.data.c_str());
      chatter_pub.publish(cmd);
      ros::spinOnce();
    }
  }
}

void simplecontrol(const pses_basis::SensorData::ConstPtr &msg,
                   ros::Publisher &chatter_pub, float *currentVelPtr,
                   bool *mode) {
  command_data cmd;

  if (*mode == true) {
    float ldist = msg->range_sensor_left;
    float rdist = msg->range_sensor_right;
    float currentRange = msg->range_sensor_front;
    float solldist = 0.8;
    float steerfact = 10;

    /* 2Punktregler
    cmd.motor_level= 5;
    if(ldist>rdist){    cmd.steering_level=  20;}
    if(rdist>ldist){	cmd.steering_level= -20;}
    */

    // P-Regler, tb = 62s

    cmd.motor_level = 5;
    float p = -8;

    float e = solldist - ldist;
    cmd.steering_level = steerfact * p * e;

    /*PID-Geschwindigkeitsalgorithmus, tb = ?
    cdmotor_level= 5;

    static float esum = 0;
    static ealt = 0;

    float p = -8;
    fload i = 1;
    float d = 1;

    float e = solldist - ldist;

    esum = esum + e;

    cmd.steering_level=steerfact*(p*e+i*esum+d*(e-ealt))

    ealt = e;
    */

    

    if (currentRange < 0.5 && *currentVelPtr > 0) {
      cmd.motor_level = 0;
    }
    // ROS_INFO("%s", retmsg.data.c_str());
    cmd.header.stamp = ros::Time::now();
    chatter_pub.publish(cmd);
    ros::spinOnce();

  } else {
    cmd.motor_level = 0;
    cmd.steering_level = 0;
    cmd.header.stamp = ros::Time::now();
    chatter_pub.publish(cmd);
    ros::spinOnce();
  }
}

void getMode(const std_msgs::String::ConstPtr &msg, bool *mode) {

  if (msg->data == "Follow Wall") {
    *mode = true;
  } else {
    *mode = false;
  }
}

// %EndTag(CALLBACK)%

void getCurrentVelocity(const command_data::ConstPtr &msg,
                        float *currentVelPtr) {
  *currentVelPtr = msg->motor_level;
}
int main(int argc, char **argv) {

  float currentVel = 1;
  bool mode = false; // false falls es nicht Startzustand ist

  /**
 * The ros::init() function needs to see argc and argv so that it can perform
 * any ROS arguments and name remapping that were provided at the command line.
 * For programmatic remappings you can use a different version of init() which
 * takes
 * remappings directly, but for most command-line programs, passing argc and
 * argv is
 * the easiest way to do it.  The third argument to init() is the name of the
 * node.
 *
 * You must call one of the versions of ros::init() before using any other
 * part of the ROS system.
 */
  ros::init(argc, argv, "listener");

  /**
 * NodeHandle is the main access point to communications with the ROS system.
 * The first NodeHandle constructed will fully initialize this node, and the
 * last
 * NodeHandle destructed will close down the node.
 */
  ros::NodeHandle n;

  ros::Publisher chatter_pub =
      n.advertise<command_data>("pses_basis/command", 1000);

  /**
 * The subscribe() call is how you tell ROS that you want to receive messages
 * on a given topic.  This invokes a call to the ROS
 * master node, which keeps a registry of who is publishing and who
 * is subscribing.  Messages are passed to a callback function, here
 * called chatterCallback.  subscribe() returns a Subscriber object that you
 * must hold on to until you want to unsubscribe.  When all copies of the
 * Subscriber
 * object go out of scope, this callback will automatically be unsubscribed from
 * this topic.
 *
 * The second parameter to the subscribe() function is the size of the message
 * queue.  If messages are arriving faster than they are being processed, this
 * is the number of messages that will be buffered up before beginning to throw
 * away the oldest ones.
 */
  // %Tag(SUBSCRIBER)#
  ros::Subscriber submode = n.subscribe<std_msgs::String>(
      "pses_basis/mode_control", 1000,
      std::bind(getMode, std::placeholders::_1, &mode));
  ros::Subscriber cmd_sub = n.subscribe<command_data>(
      "pses_basis/command", 1000,
      std::bind(getCurrentVelocity, std::placeholders::_1, &currentVel));
  ros::Subscriber sub = n.subscribe<pses_basis::SensorData>(
      "pses_basis/sensor_data", 1000,
      std::bind(simplecontrol, std::placeholders::_1, chatter_pub, &currentVel,
                &mode));

  // %EndTag(SUBSCRIBER)%

  /**
 * ros::spin() will enter a loop, pumping callbacks.  With this version, all
 * callbacks will be called from within this thread (the main one).  ros::spin()
 * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
 */
  // %Tag(SPIN)%
  ros::spin();
  // %EndTag(SPIN)%

  return 0;
}
// %EndTag(FULLTEXT)%
