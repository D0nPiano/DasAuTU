#ifndef PDController_H
#define PDController_H

#include "ros/ros.h"
/**
*@brief implements a PD Controller
*/
class PDController {
public:
  PDController() {}
  PDController(ros::NodeHandle &nh);
  PDController(ros::NodeHandle &nh, float p, float d, int maxMotorLevel,
            float solldist);
  void reset();
  void drive(float ldist, bool left);

  void setMaxMotorLevel(const int16_t &value);

  int16_t getMaxMotorLevel() const;

private:
  float e0;
  double t0;
  int16_t maxMotorLevel;
  float p;
  float d;
  float solldist;
  ros::Publisher command_pub;
};

#endif // PDController_H
