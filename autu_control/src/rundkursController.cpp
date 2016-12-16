#ifndef _RundkursController_CPP_
#define _RundkursController_CPP_

#include "ros/ros.h"
#include "autu_control/rundkursController.h"

RundkursController::RundkursController(ros::NodeHandle * n, ros::Publisher *command_pub): n(n), command_pub(command_pub){
	ROS_INFO("New RundkursController");

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

	laserDetector = new LaserDetector(currentLaserScan);

	drivingCurve = false;
}

RundkursController::~RundkursController()
{
	ROS_INFO("Destroying RundkursController");
	this->stop();
	laser_sub.shutdown();
	sensor_sub.shutdown();
}

void RundkursController::driveCurve(){
	command_data cmd;

	static float driveStraightTime;
	static float driveCurveTime;
	static float cornerBeginAngle;

	float curveTimer = ros::Time::now().toSec() - curveBegin;

	if(curveTimer < 0.3){
		float ldist = currentSensorData->range_sensor_left;
		driveStraightTime = 0.3 + (1.5 * ldist);
		cornerBeginAngle = laserDetector->getAngleToWall();

		float curveSeconds = 1.0 + (cornerBeginAngle / PI / 2) * 9.0;
		driveCurveTime = driveStraightTime + .6 + curveSeconds;

		//ROS_INFO("CurveCompleted: cornerBeginAngle: [%f]", (cornerBeginAngle  * 180 / PI));
		//ROS_INFO("CurveCompleted: driveStraightTime: [%f]", driveStraightTime);
		//ROS_INFO("CurveCompleted: driveCurveTime: [%f]", driveCurveTime);	
	}

	if(curveTimer < driveStraightTime) {
    	cmd.motor_level = 10;
    	cmd.steering_level = 0;
	} else if(curveTimer > driveCurveTime - 1.1 && curveTimer < driveCurveTime && laserDetector->getAngleToWallInDeg() < 95.0){
    	ROS_INFO("Kurvenfahrt beendet");
    	cmd.motor_level = 10;
    	cmd.steering_level = 0;
  		drivingCurve = false;
	} else if(curveTimer < driveCurveTime){
    	cmd.motor_level = 7;
    	cmd.steering_level = 30;
	} else {
    	cmd.motor_level = 10;
    	cmd.steering_level = 0;
  		drivingCurve = false;
	}

	cmd.header.stamp = ros::Time::now();
	command_pub->publish(cmd);
	ros::spinOnce();
}

void RundkursController::driveStraight(){
	command_data cmd;

	float ldist = currentSensorData->range_sensor_left;
	ldist += laserDetector->getDistanceToWall();
	ldist = ldist / 2.0;

	/* TODO: Get corner, set 
			drivingCurve = true;
			curveBegin = ros::Time::now().toSec();
			*/

	float solldist = 0.6;
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
	command_pub->publish(cmd);
	ros::spinOnce();
}

void RundkursController::stop(){
	command_data cmd;
	cmd.motor_level = 0;
	cmd.steering_level = 0;
	cmd.header.stamp = ros::Time::now();
	command_pub->publish(cmd);
	ros::spinOnce();
}

void RundkursController::getCurrentLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg){
	*currentLaserScan = *msg;
}

void RundkursController::getCurrentSensorData(const pses_basis::SensorData::ConstPtr&  msg){
	*currentSensorData = *msg;
}

void RundkursController::simpleController(){
	laserDetector->getDistanceToWall();
	if(drivingCurve){
		this->driveCurve();
		if(		laserDetector->isNextToWall()
				&& (curveBegin + RundkursController_MAX_CURVE_SECONDS) < ros::Time::now().toSec()
				&& (laserDetector->getAngleToWall() * 180 / PI) < 100.0){
			drivingCurve = false;
		}
	} else {
		this->driveStraight();
		if(laserDetector->isNextToCorner()){
			ROS_INFO("************ Corner ***************");
			drivingCurve = true;
			curveBegin = ros::Time::now().toSec();
			ROS_INFO("Seconds: [%f]", curveBegin);
		}
	}
	//ROS_INFO("Angle to wall in deg: [%f]", laserDetector->getAngleToWall() * 180 / PI);
}

void RundkursController::run() {
	if(!initialized){
		if(currentLaserScan->ranges[0] == -1.0 || currentSensorData->range_sensor_left == -1.0){
			ROS_INFO("Sensors Uninitialized!");
			return;
		} else {
			initialized = true;
			laserDetector->initialize();
		}
	}

	//If everything is initialized, run Controller
	this->simpleController();
}

#endif