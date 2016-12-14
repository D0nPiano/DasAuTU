#ifndef _LaserDetector_H_
#define _LaserDetector_H_

#include "ros/ros.h"
#include "pses_basis/SensorData.h"

#include <sensor_msgs/LaserScan.h>

class LaserDetector {
   public:
   		LaserDetector(sensor_msgs::LaserScan *);
   		~LaserDetector();
         void initialize();
   		bool isNextToCorner();
   		bool isNextToWall();
   		float getAngleToWall();
   		float getAngleToWallInDeg();
   		float getDistanceToCorner();
   private:
   		float calculateBeta(int angleBegin, float & alpha, float & epsilon);
   		sensor_msgs::LaserScan *currentLaserScan;
   		int RANGE_START;
   		int RANGE_DIFF;
   		float CORNER_SENSITIVITY;
};

#endif