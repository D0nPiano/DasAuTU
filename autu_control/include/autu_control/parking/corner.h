#ifndef CORNER_H
#define CORNER_H

#include "autu_control/parking/line.h"

#include "Eigen/Dense"

#include "nav_msgs/Path.h"
#include "ros/ros.h"

class Corner {
public:
  Corner() {}
  Corner(const Eigen::Vector2f &a, const Eigen::Vector2f &b,
         const Eigen::Vector2f &c);
  Corner(const Line &line1, const Line &line2);
  nav_msgs::Path toPathMsg1() const;
  nav_msgs::Path toPathMsg2() const;

  Eigen::Vector2f getB() const;

private:
  Eigen::Vector2f a;
  Eigen::Vector2f b;
  Eigen::Vector2f c;
  Eigen::ParametrizedLine<float, 2> x_line;
  Eigen::ParametrizedLine<float, 2> y_line;
};

#endif // CORNER_H
