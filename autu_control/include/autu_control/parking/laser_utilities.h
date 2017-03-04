#ifndef LASER_UTILITIES_H
#define LASER_UTILITIES_H

#include <vector>

#include "Eigen/Dense"
#include "Eigen/StdVector"

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "autu_control/parking/corner.h"
#include "autu_control/rundkurs/line.h"

class LaserUtil {
public:
  LaserUtil(ros::NodeHandle &nh);
  Eigen::Vector2f findCorner(const sensor_msgs::LaserScanConstPtr &scan);
  float getDistanceToWall(const sensor_msgs::LaserScanConstPtr &scan,
                          bool left);
  float getAngleToWallRLF(const sensor_msgs::LaserScanConstPtr &scan,
                          bool left);
  float calcCornerSize(const sensor_msgs::LaserScanConstPtr &scan,
                       const Eigen::Vector2f &corner, bool left);

private:
  Eigen::ParametrizedLine<float, 2> findLine(
      const std::vector<Eigen::Vector2f,
                        Eigen::aligned_allocator<Eigen::Vector2f>> &points);
  Corner findCornerRLF(
      const std::vector<Eigen::Vector2f,
                        Eigen::aligned_allocator<Eigen::Vector2f>> &points);
  std::vector<Line> findLinesRLF(
      const std::vector<Eigen::Vector2f,
                        Eigen::aligned_allocator<Eigen::Vector2f>> &points);
  std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>
  filterScan(const sensor_msgs::LaserScanConstPtr &scan, bool left);

  float delta_max;
  float dist_corner_to_line;
#ifndef NDEBUG
  ros::Publisher wall_pub;
  ros::Publisher corner1_pub;
  ros::Publisher corner2_pub;
#endif
};

#endif // LASER_UTILITIES_H