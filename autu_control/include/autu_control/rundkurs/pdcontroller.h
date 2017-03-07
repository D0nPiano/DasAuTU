#ifndef PDController_H
#define PDController_H

#include "ros/ros.h"
/**
*@brief implements a PD Controller
*Controls steering level to keep a constant distace to a wall
*Publishes steering level on autu/command
*/
class PDController {
public:
  PDController() {}
  /**
  *@brief Constructor using p d values and motor Level published on ros param server 
  */
  PDController(ros::NodeHandle &nh);
   /**
  *@brief Constructor using passed p d values and motor Level  
  */
  PDController(ros::NodeHandle &nh, float p, float d, int maxMotorLevel,
            float solldist);
  void reset();

  /**
  *@brief run control iterration
  *@param ldist current distance to wall
  *@param left true if controlling distance to left wall
  */
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
