#ifndef _RundkursController_H_
#define _RundkursController_H_

/*
	If LaserScan is uninitialized, it's range[0] is -1.0
	If SensorData is uninitialized, it's range_sensor_left is -1.0
*/


#include "autu_control/AutoController.h"

#include <iostream>
#include <exception>
#include "std_msgs/String.h"

#include "ros/ros.h"
#include "pses_basis/SensorData.h"
#include <pses_basis/Command.h>
typedef pses_basis::Command command_data;

#include <sensor_msgs/LaserScan.h>

class RundkursController : public AutoController  {
   public:
   		RundkursController(ros::NodeHandle * n);
   		~RundkursController();
    	void run();
   private:
   		void driveStraight();
   		void getCurrentLaserScan(const sensor_msgs::LaserScan::ConstPtr& );
   		void getCurrentSensorData(const pses_basis::SensorData::ConstPtr& );
   		void simpleController();
   		ros::NodeHandle * n;
   		ros::Publisher command_pub;
   		ros::Subscriber laser_sub;
   		ros::Subscriber sensor_sub;
   		sensor_msgs::LaserScan *currentLaserScan;
   		pses_basis::SensorData *currentSensorData;
};

RundkursController::RundkursController(ros::NodeHandle * n): n(n){
	ROS_INFO("New RundkursController");

	command_pub = n->advertise<command_data>("pses_basis/command", 1000);

	laser_sub = n->subscribe<sensor_msgs::LaserScan>(
      "/scan", 1000, &RundkursController::getCurrentLaserScan, this);

	sensor_sub = n->subscribe<pses_basis::SensorData>(
      "pses_basis/sensor_data", 1000, &RundkursController::getCurrentSensorData, this);


	// Initialite LaserScan
	currentLaserScan = new sensor_msgs::LaserScan;
	std::vector<float> tmpRanges;
	tmpRanges.push_back(-1.0);
	currentLaserScan->ranges = tmpRanges;

	// Initialize SensorData
	currentSensorData = new pses_basis::SensorData;
	currentSensorData->range_sensor_left = -1.0;
}

RundkursController::~RundkursController()
{
	ROS_INFO("Destroying RundkursController");
	laser_sub.shutdown();
	sensor_sub.shutdown();
}

void RundkursController::driveStraight(){
	command_data cmd;

	float ldist = currentSensorData->range_sensor_left;
	float rdist = currentSensorData->range_sensor_right;
	float currentRange = currentSensorData->range_sensor_front;

	float solldist = 0.7;
	float steerfact = -2;
	static float e0 = 0;
	static double t0 = 0;

	// P-Regler, tb = 62s

	cmd.motor_level = 20;
	float p = 18;
	float d = 10;
	double t = ros::Time::now().toSec();
	float e = solldist - ldist;
	cmd.steering_level = steerfact *  p *( e+ (e-e0)*d/(t-t0));

	e0 = e;
	t0 = t;

	if(cmd.steering_level > 40)
	cmd.steering_level = 40;
	else if (cmd.steering_level < -40)
	cmd.steering_level = -40;

	cmd.header.stamp = ros::Time::now();
	command_pub.publish(cmd);
	ros::spinOnce();
}

void RundkursController::getCurrentLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg){
	*currentLaserScan = *msg;
}

void RundkursController::getCurrentSensorData(const pses_basis::SensorData::ConstPtr&  msg){
	*currentSensorData = *msg;
}

void RundkursController::simpleController(){
  this->driveStraight();
}

void RundkursController::run() {
	if(currentLaserScan->ranges[0] == -1.0 || currentSensorData->range_sensor_left == -1.0){
		ROS_INFO("Uninitialized!");
		return;
	}
	//If everything is initialized, run Controller
	this->simpleController();
}


#endif