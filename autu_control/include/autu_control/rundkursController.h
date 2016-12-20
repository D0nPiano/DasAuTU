#ifndef _RundkursController_H_
#define _RundkursController_H_


#define PI 3.14159265

#define RundkursController_MAX_CURVE_SECONDS 8.0

/*
	If LaserScan is uninitialized, its range[0] is -1.0
	If SensorData is uninitialized, its range_sensor_left is -1.0
*/


#include "autu_control/AutoController.h"
#include "autu_control/laserDetector.h"

#include <iostream>
#include <exception>
#include "std_msgs/String.h"

#include <sensor_msgs/LaserScan.h>

#include "ros/ros.h"
#include "pses_basis/SensorData.h"
#include <pses_basis/Command.h>
typedef pses_basis::Command command_data;

class RundkursController : public AutoController  {
   public:
   		RundkursController(ros::NodeHandle * n, ros::Publisher *command_pub);
   		~RundkursController();
    	void run();
   private:
   		void driveCurve();
   		void driveStraight();
   		void stop();
   		void getCurrentLaserScan(const sensor_msgs::LaserScan::ConstPtr& );
   		void getCurrentSensorData(const pses_basis::SensorData::ConstPtr& );
   		void simpleController();
         void beginCurve();
   		ros::NodeHandle * n;
   		ros::Publisher *command_pub;
   		ros::Subscriber laser_sub;
   		ros::Subscriber sensor_sub;
   		sensor_msgs::LaserScan *currentLaserScan;
   		pses_basis::SensorData *currentSensorData;
   		LaserDetector *laserDetector;
   		bool drivingCurve;
   		double curveBegin;
   		bool initialized;
};

#endif