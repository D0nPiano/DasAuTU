#ifndef CORNER_H
#define CORNER_H

#include "Eigen/Dense"

#include "nav_msgs/Path.h"
#include "ros/ros.h"

class Corner {
public:
  Corner();
  Corner(const Eigen::Vector2f &origin, const Eigen::Vector2f &direction_x,
         const Eigen::Vector2f &direction_y);
  float distance(const Eigen::Vector2f &point) const;
  nav_msgs::Path toPath1() const;
  nav_msgs::Path toPath2() const;

  Eigen::Vector2f getOrigin() const;

private:
  Eigen::Vector2f origin;
  Eigen::Vector2f direction_x;
  Eigen::Vector2f direction_y;
  Eigen::ParametrizedLine<float, 2> x_line;
  Eigen::ParametrizedLine<float, 2> y_line;
};

#endif // CORNER_H
