#ifndef CORNER_H
#define CORNER_H

#include "Eigen/Dense"
#include "autu_control/parking/line.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"

/**
 * @brief Representation of a corner
 *
 * A corner consists of two lines with a common origin.
 * The first line goes from point a to point b.
 * The second line goes from point b to point c.
 */
class Corner {
public:
  Corner() {}

  /**
   * @brief Constructs a corner from the given points.
   * @param a - The first outer point
   * @param b - The point in the center of the corner
   * @param c - The other outer point
   */
  Corner(const Eigen::Vector2f &a, const Eigen::Vector2f &b,
         const Eigen::Vector2f &c);

  /**
   * @brief Constructs a corner from the given lines.
   *
   * The lines have to have one common origin.
   */
  Corner(const Line &line1, const Line &line2);

  /**
   * @brief Generates a message for the first line for RVIZ.
   */
  nav_msgs::Path toPathMsg1() const;

  /**
   * @brief Generates a message for the second line for RVIZ.
   */
  nav_msgs::Path toPathMsg2() const;

  /**
   * @brief Getter for the common origin of both lines
   */
  Eigen::Vector2f getB() const;

private:
  /**
   * @brief End point of first line
   */
  Eigen::Vector2f a;

  /**
   * @brief Common origin of both lines
   */
  Eigen::Vector2f b;

  /**
   * @brief End point of second line
   */
  Eigen::Vector2f c;
};

#endif // CORNER_H
