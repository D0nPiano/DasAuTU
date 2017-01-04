#ifndef LASER_UTILITIES_H
#define LASER_UTILITIES_H

#include <vector>

#include "Eigen/Dense"
#include "Eigen/StdVector"

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class LaserUtil {
public:
  LaserUtil(ros::NodeHandle &nh);
  Eigen::ParametrizedLine<float, 2> findLine(
      const std::vector<Eigen::Vector2f,
                        Eigen::aligned_allocator<Eigen::Vector2f>> &points);
  float getDistanceToWall(const sensor_msgs::LaserScanConstPtr &scan,
                          bool left);

private:
  ros::Publisher wall_pub;
};

#endif // LASER_UTILITIES_H
