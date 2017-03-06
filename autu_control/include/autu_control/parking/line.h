#ifndef LINE_H
#define LINE_H

#include "Eigen/Dense"
#include "nav_msgs/Path.h"

/**
 * @brief Represents a simple line with an origin and an end point.
 */
class Line {
public:
  /**
 * @brief Constructs a line from 2 given points.
 * @param first - Starting point of the line
 * @param last - End point of the line
 */
  Line(const Eigen::Vector2f &first, const Eigen::Vector2f &last);

  /**
   * @brief Generates a nav_msgs::Path representation of the line to be shown in
   * RVIZ.
   * @return The generated message
   */
  nav_msgs::Path toPathMsg() const;

  /**
   * @brief Determines if both lines have a common origin.
   * @return True = common origin exists
   */
  bool hasPointInCommon(const Line &other) const;

  /**
   * @brief Returns the Eigen representation of the line.
   * @return
   */
  Eigen::ParametrizedLine<float, 2> toEigenLine() const;

  /**
   * @brief Getter for the origin point
   */
  Eigen::Vector2f getOrigin() const;

  /**
   * @brief Getter for the end point
   */
  Eigen::Vector2f getEnd() const;

private:
  /**
   * @brief origin point of the line
   */
  Eigen::Vector2f origin;

  /**
   * @brief end point of the line
   */
  Eigen::Vector2f end;
};

#endif // LINE_H
