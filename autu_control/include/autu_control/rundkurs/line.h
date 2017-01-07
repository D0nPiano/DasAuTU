#ifndef LINE_H
#define LINE_H

#include "Eigen/Dense"
#include "nav_msgs/Path.h"

class Line {
public:
  Line(const Eigen::Vector2f &first, const Eigen::Vector2f &last);
  nav_msgs::Path toPathMsg() const;

private:
  Eigen::Vector2f origin;
  Eigen::Vector2f end;
};

#endif // LINE_H
