#include "autu_control/rundkurs/curvedriver.h"

#include <math.h>

#include "pses_basis/Command.h"

#define MAX_CURVE_SECONDS 8.0

CurveDriver::CurveDriver(ros::NodeHandle &nh) {
  command_pub = nh.advertise<pses_basis::Command>("autu/command", 1);
}

void CurveDriver::reset() {
  int currentTime = ros::Time::now().toSec();
  if (currentTime - curveBegin > 6) {
    ROS_INFO("Beginning Curve");
    curveBegin = ros::Time::now().toSec();
  } else {
    ROS_INFO("Curve was not too long ago...");
  }
}

void CurveDriver::drive(float ldist, float angleToWall) {
  pses_basis::Command cmd;

  float curveTimer = ros::Time::now().toSec() - curveBegin;

  if (curveTimer < 0.3) {
    // float ldist = currentSensorData->range_sensor_left;
    driveStraightTime = 0.0 + (0.5 * ldist);
    cornerBeginAngle = angleToWall;
    // driveStraightTime = 0.0;

    float curveSeconds = 0.4 + (cornerBeginAngle / M_PI / 2) * 6.0;
    driveCurveTime = driveStraightTime + .6 + curveSeconds;

    ROS_INFO("CurveCompleted: cornerBeginAngle: [%f]",
             (cornerBeginAngle * 180 / M_PI));
    ROS_INFO("CurveCompleted: driveStraightTime: [%f]", driveStraightTime);
    ROS_INFO("CurveCompleted: driveCurveTime: [%f]", driveCurveTime);
  }

  if (curveTimer < driveStraightTime) {
    cmd.motor_level = 10;
    cmd.steering_level = 0;
  } else if (curveTimer > driveCurveTime - 1.1 && curveTimer < driveCurveTime &&
             (angleToWall * 180 / M_PI) < 95.0) {
    ROS_INFO("Kurvenfahrt beendet");
    cmd.motor_level = 10;
    cmd.steering_level = 0;
  } else if (curveTimer < driveCurveTime) {
    cmd.motor_level = 7;
    cmd.steering_level = 30;
  } else {
    cmd.motor_level = 10;
    cmd.steering_level = 0;
  }

  cmd.header.stamp = ros::Time::now();
  command_pub.publish(cmd);
  ros::spinOnce();
}

bool CurveDriver::isAroundTheCorner() {
  return (curveBegin + MAX_CURVE_SECONDS) < ros::Time::now().toSec();
}
