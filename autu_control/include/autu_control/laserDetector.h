#ifndef _LaserDetector_H_
#define _LaserDetector_H_

#include <sensor_msgs/LaserScan.h>

#define PI 3.14159265
#define ANGLE_OFFSET 0.0

class LaserDetector {
   public:
   		LaserDetector(sensor_msgs::LaserScan *);
   		~LaserDetector();
   		bool isNextToCorner();
   		bool isNextToWall();
   		float getAngleToWall();
   		float getAngleToWallInDeg();
   private:
   		float calculateBeta(int angleBegin, float & alpha, float & epsilon);
   		sensor_msgs::LaserScan *currentLaserScan;
   		int RANGE_START;
   		int RANGE_DIFF;
   		float CORNER_SENSITIVITY;
};

LaserDetector::LaserDetector(sensor_msgs::LaserScan * laserPtr): currentLaserScan(laserPtr){
	ROS_INFO("New LaserDetector");
	CORNER_SENSITIVITY = 1.3;
	RANGE_START = 349;
	RANGE_DIFF = 10;
}

LaserDetector::~LaserDetector()
{
	ROS_INFO("Destroying LaserDetector");
}

bool LaserDetector::isNextToCorner(){
	float laserDiffFront = currentLaserScan->ranges[RANGE_START - RANGE_DIFF*2] - currentLaserScan->ranges[RANGE_START - RANGE_DIFF];
	float laserDiffBack = currentLaserScan->ranges[RANGE_START - RANGE_DIFF] - currentLaserScan->ranges[RANGE_START];
	bool retVal = laserDiffFront > (1.3 / CORNER_SENSITIVITY) && laserDiffBack < (0.3 * CORNER_SENSITIVITY);
	return retVal;
}

bool LaserDetector::isNextToWall(){
	if(this->isNextToCorner())
		return false;

	float laserDiffFront = currentLaserScan->ranges[RANGE_START - RANGE_DIFF*3] - currentLaserScan->ranges[RANGE_START - RANGE_DIFF*2];
	float laserDiffBack = currentLaserScan->ranges[RANGE_START - RANGE_DIFF * 1] - currentLaserScan->ranges[RANGE_START - RANGE_DIFF * 2];
	bool retVal = currentLaserScan->ranges[RANGE_START - RANGE_DIFF*3] < 3.0 && currentLaserScan->ranges[RANGE_START - RANGE_DIFF*2] < 2.8 && currentLaserScan->ranges[RANGE_START - RANGE_DIFF*1] < 2.6 && laserDiffFront < 0.7 && laserDiffBack < 0.5;
	return retVal;
}

float LaserDetector::calculateBeta(int angleBegin, float & alpha, float & epsilon){

	float b = currentLaserScan->ranges[angleBegin];
	float c = currentLaserScan->ranges[angleBegin - RANGE_DIFF];
	float a = sqrt(b*b+c*c-2*b*c*cos(alpha));
	float beta = asin(b*sin(alpha)/a);

	//float angleToWall = PI - beta - epsilon;
	//ROS_INFO("Angle to wall in deg: [%f]", angleToWall * 180 / PI);
	return beta;
}


float LaserDetector::getAngleToWall(){
	float angle_range = currentLaserScan->angle_max - currentLaserScan->angle_min; // 70 deg
	float alpha = RANGE_DIFF * currentLaserScan->angle_increment; // angle alpha used for Calculation in rad
	float epsilon = PI/2 - angle_range/2 + alpha;

	float beta1 = this->calculateBeta(RANGE_START, alpha, epsilon);
	float beta2 = this->calculateBeta(RANGE_START - ((int)RANGE_DIFF*0.333), alpha, epsilon);
	float beta3 = this->calculateBeta(RANGE_START - ((int)RANGE_DIFF*0.666), alpha, epsilon);
	float beta4 = this->calculateBeta(RANGE_START - ((int)RANGE_DIFF*1.0), alpha, epsilon);
	float beta5 = this->calculateBeta(RANGE_START - ((int)RANGE_DIFF*1.333), alpha, epsilon);

	float beta = (beta1 + beta2 + beta3 + beta4 + beta5) / 5.0;

	//ROS_INFO("beta in deg: [%f]", beta * 180 / PI);

	float angleToWall = PI - beta - epsilon - ANGLE_OFFSET;
	//ROS_INFO("Angle to wall in deg: [%f]", angleToWall * 180 / PI);
	return angleToWall;
}

float LaserDetector::getAngleToWallInDeg(){
	float angleToWall = this->getAngleToWall();
	return (angleToWall * 180.0 / PI);
}

#endif