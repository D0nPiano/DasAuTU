#ifndef LASER_UTILITIES_H
#define LASER_UTILITIES_H

#include "Eigen/Dense"
#include "Eigen/StdVector"
#include "autu_control/parking/corner.h"
#include "autu_control/parking/line.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>

/**
 * @brief Contains methods to analyze laserscans.
 */
class LaserUtil {
public:
  LaserUtil(ros::NodeHandle &nh);

  /**
   * @brief Detects a corner in the given laserscan.
   *
   * A corner is defined as 2 lines which have a common origin.
   * @return if no corner was found (0,0) is returned
   */
  Eigen::Vector2f findCorner(const sensor_msgs::LaserScanConstPtr &scan);

private:
  /**
   * @brief Detects a corner in the given laserscan.
   *
   * A corner is defined as 2 lines which have a common origin.
   * @return if no corner was found (0,0) is returned
   */
  Corner findCornerRLF(
      const std::vector<Eigen::Vector2f,
                        Eigen::aligned_allocator<Eigen::Vector2f>> &points);

  /**
   * @brief Detects lines in the given set of points via Recursive-Line-Fitting.
   * @return All lines which have been found
   */
  std::vector<Line> findLinesRLF(
      const std::vector<Eigen::Vector2f,
                        Eigen::aligned_allocator<Eigen::Vector2f>> &points);

  /**
   * @brief Converts the given laserscan to a set of points in cartesian
   * coordinates.
   *
   * Only the left or the right half of the scan is used.
   *
   * @param scan - The given laserscan
   * @param left - Defines which half of the scan is used.
   * @return The filtered points
   */
  std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>
  convertToCartesian(const sensor_msgs::LaserScanConstPtr &scan, bool left);

  /**
   * @brief Error threshold for RLF
   *
   * If a point's distance to the current line is bigger than this threshold,
   * split the line at this point.
   */
  float deltaMax;

  /**
   * @brief Lines which consist of less points will be discarded.
   */
  size_t minimumLineSize;

  /**
   * @brief Used by LaserUtil::convertToCartesian to filter out points which are
   * far
   * away.
   */
  float maxWallScope;

  /**
   * @brief Used by LaserUtil::convertToCartesian to filter out points which are
   * far
   * away.
   */
  float maxWallDistance;

  /**
   * @brief Publisher for the first line of the detected corner
   */
  ros::Publisher corner1Pub;

  /**
   * @brief Publisher for the second line of the detected corner
   */
  ros::Publisher corner2Pub;
};

#endif // LASER_UTILITIES_H
