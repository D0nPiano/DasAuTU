#include "autu_control/parking/parking_controller.h"

#include "pses_basis/Command.h"
#include <cmath>

#define DETECT_CORNER 0
#define DRIVE_TO_CORNER 1
#define TURN_RIGHT_INIT 2
#define TURN_RIGHT 3
#define TURN_LEFT 4
#define STOP 5

using Eigen::Vector2f;
using Eigen::Rotation2Df;
using Eigen::aligned_allocator;
using std::abs;
using std::sqrt;
using std::sin;
using std::acos;
using std::asin;
using std::vector;
using geometry_msgs::PoseStamped;

ParkingController::ParkingController(ros::NodeHandle &nh)
    : staticTFBroadcaster(), laserUtil(nh), state(DETECT_CORNER) {

  command_pub = nh.advertise<pses_basis::Command>("autu/command", 1);

  odom_sub = nh.subscribe<nav_msgs::Odometry>(
      "/odom", 1, &ParkingController::odomCallback, this);

  laser_sub = nh.subscribe<sensor_msgs::LaserScan>(
      "/scan", 1, &ParkingController::laserscanCallback, this);

  velocity_forward = nh.param<int>("main/parking/velocity_forward", 5);
  velocity_backward = nh.param<int>("main/parking/velocity_backward", 5);

  maxSteeringLeft = nh.param<int>("main/parking/max_steering_left", 42);
  maxSteeringRight = nh.param<int>("main/parking/max_steering_right", 42);

  regulator_d = nh.param<float>("main/parking/regulator_d", 3);
  regulator_p = nh.param<float>("main/parking/regulator_p", 10);
  correction_x = nh.param<float>("main/parking/correction_x", 0.1f);

  safety_distance = nh.param<float>("main/parking/safety_distance", 0.1f);
  b = nh.param<float>("main/parking/b", 0.32f);
  w = nh.param<float>("main/parking/w", 0.2f);
  w += safety_distance;

  r = nh.param<float>("main/parking/minimal_radius", 1);

  r_e = sqrt(r * r + b * b);

  alpha = acos(1 - b * b / (4 * r * r));

  beta = asin(sin(alpha) * r / r_e);

  theta = acos((r - w) / r_e) - beta;

  delta = theta - alpha;

  pidRegler = PIDRegler(nh, regulator_p, regulator_d, velocity_forward, 0);

#ifndef NDEBUG
  trajectory_pub =
      nh.advertise<nav_msgs::Path>("autu/debug/parking_trajectory", 1);
  trajectory_odom_pub =
      nh.advertise<nav_msgs::Path>("autu/debug/parking_trajectory_odom", 1);
#endif
}

void ParkingController::odomCallback(const nav_msgs::OdometryConstPtr &msg) {
  odom = msg;
}

void ParkingController::laserscanCallback(
    const sensor_msgs::LaserScanConstPtr &msg) {
  laserscan = msg;
}

void ParkingController::run() {
  tf::Quaternion begin, current;
  if (odom == nullptr || laserscan == nullptr ||
      //   !transformListener.canTransform("/rear_right_wheel", "/odom",
      //                                 ros::Time(0)) ||
      !transformListener.canTransform("/base_laser", "/odom", ros::Time(0)))
    return;
  Vector2f vec;
  float y_dist_to_start = 0.1f;
  switch (state) {
  case DETECT_CORNER:
    vec = laserUtil.findCorner(laserscan);

    try {
      tf::StampedTransform transform;
      transformListener.waitForTransform("base_laser", "/odom", ros::Time(0),
                                         ros::Duration(0.1));
      transformListener.lookupTransform("base_laser", "/odom", ros::Time(0),
                                        transform);

      geometry_msgs::PointStamped cornerInBaseLaser, cornerInOdom;

      vec[1] += safety_distance;
      cornerInBaseLaser.point.x = vec[0];
      cornerInBaseLaser.point.y = vec[1];
      cornerInBaseLaser.header.frame_id = "/base_laser";
      cornerInBaseLaser.header.stamp = ros::Time(0);
      transformListener.transformPoint("/odom", cornerInBaseLaser,
                                       cornerInOdom);
      cornerInOdom.point.z = 0;

      /* tf::Transform transformToCorner;
       transformToCorner.setOrigin(
           tf::Vector3(cornerInOdom.point.x, cornerInOdom.point.y, 0.0));
       tf::Quaternion q(0, 0, 0, 1);
       transformToCorner.setRotation(q);
         transformBroadcaster.sendTransform(
           tf::StampedTransform(transformToCorner, ros::Time::now(), "/odom",
                                "/parking"));*/

      geometry_msgs::TransformStamped transformToCorner;
      transformToCorner.transform.translation.x = cornerInOdom.point.x;
      transformToCorner.transform.translation.y = cornerInOdom.point.y;
      transformToCorner.transform.translation.z = 0;
      transformToCorner.transform.rotation.x = 0;
      transformToCorner.transform.rotation.y = 0;
      transformToCorner.transform.rotation.z = 0;
      transformToCorner.transform.rotation.w = 1;
      transformToCorner.child_frame_id = "parking";
      transformToCorner.header.frame_id = "odom";
      staticTFBroadcaster.sendTransform(transformToCorner);

      start.point.x = r * sin(delta);
      start.point.y = r * (1 - cos(delta));
      start.header.frame_id = "/parking";
      start.header.stamp = ros::Time(0);
      start.point.z = 0;
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }
#ifndef NDEBUG
    publishParkingTrajectory();
#endif
    state = DRIVE_TO_CORNER;
    break;
  case DRIVE_TO_CORNER:

    try {
      tf::StampedTransform transform;
      transformListener.waitForTransform("/parking", "/rear_right_wheel",
                                         ros::Time(0), ros::Duration(0.1));
      transformListener.lookupTransform("/parking", "/rear_right_wheel",
                                        ros::Time(0), transform);

      geometry_msgs::PointStamped startInRightWheel;
      start.header.stamp = ros::Time(0);
      transformListener.transformPoint("/rear_right_wheel", start,
                                       startInRightWheel);
      ROS_INFO("x: %f", startInRightWheel.point.x);
      y_dist_to_start = -startInRightWheel.point.y;
      if (startInRightWheel.point.x < correction_x)
        state = TURN_RIGHT_INIT;
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }
    break;
  case TURN_RIGHT_INIT:
    curveBegin = odom->pose.pose;
    state = TURN_RIGHT;
    break;
  case TURN_RIGHT:
    tf::quaternionMsgToTF(curveBegin.orientation, begin);
    tf::quaternionMsgToTF(odom->pose.pose.orientation, current);
    if (2 * begin.angle(current) > theta) {
      curveBegin = odom->pose.pose;
      state = TURN_LEFT;
    }
    break;
  case TURN_LEFT:
    tf::quaternionMsgToTF(curveBegin.orientation, begin);
    tf::quaternionMsgToTF(odom->pose.pose.orientation, current);
    if (2 * begin.angle(current) > theta)
      state = STOP;
    break;
  default:
    break;
  }

  pses_basis::Command cmd;
  switch (state) {
  case DRIVE_TO_CORNER:
    pidRegler.drive(y_dist_to_start, false);
    break;
  case TURN_RIGHT:
    cmd.motor_level = -velocity_backward;
    cmd.steering_level = -maxSteeringRight;
    command_pub.publish(cmd);
    break;
  case TURN_LEFT:
    cmd.motor_level = -velocity_backward;
    cmd.steering_level = maxSteeringLeft;
    command_pub.publish(cmd);
    break;
  case STOP:
    cmd.motor_level = 0;
    cmd.steering_level = maxSteeringLeft;
    command_pub.publish(cmd);
    break;
  default:
    break;
  }
}

std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>
ParkingController::calcTrajectory() {
  vector<Vector2f, aligned_allocator<Vector2f>> vectors;
  const Vector2f start(r * sin(delta), r * (1 - cos(delta)));

  const Vector2f v_r(0, r);

  const Vector2f startICC = start - v_r;

  vectors.push_back(start);

  const int intervals = 20;
  for (int i = 0; i <= intervals; ++i) {
    Rotation2Df rot(theta * i / intervals);

    vectors.push_back(rot * v_r + startICC);
  }

  const Vector2f endICC = Rotation2Df(theta) * (2 * v_r) + startICC;

  for (int i = intervals; i >= 0; --i) {
    Rotation2Df rot(theta * i / intervals);

    vectors.push_back(rot * -v_r + endICC);
  }
  return vectors;
}

void ParkingController::publishParkingTrajectory() {
  nav_msgs::Path msgWheel, msgOdom;
  msgWheel.header.frame_id = "parking";
  msgOdom.header.frame_id = "parking";

  const auto &vectors = calcTrajectory();

  for (const Vector2f &vec : vectors) {
    PoseStamped pose;
    pose.pose.position.x = vec[0]; // + corner.point.x;
    pose.pose.position.y = vec[1]; // + corner.point.y;
    msgWheel.poses.push_back(pose);

    pose.pose.position.y += w;
    msgOdom.poses.push_back(pose);
  }

  trajectory_pub.publish(msgWheel);
  trajectory_odom_pub.publish(msgOdom);
}
