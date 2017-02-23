#include "autu_control/parking/parking_controller.h"

#include "pses_basis/Command.h"
#include <cmath>

// states of the parking controller
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

  commandPub = nh.advertise<pses_basis::Command>("autu/command", 1);

  parkingSpotPub = nh.advertise<nav_msgs::Path>("autu/parking_spot", 1);

  odomSub = nh.subscribe<nav_msgs::Odometry>(
      "/odom", 1, &ParkingController::odomCallback, this);

  laserscanSub = nh.subscribe<sensor_msgs::LaserScan>(
      "/scan", 1, &ParkingController::laserscanCallback, this);

  velocityForward = nh.param<int>("main/parking/velocity_forward", 5);
  velocityBackward = nh.param<int>("main/parking/velocity_backward", 5);

  maxSteeringLeft = nh.param<int>("main/parking/max_steering_left", 42);
  maxSteeringRight = nh.param<int>("main/parking/max_steering_right", 42);

  regulatorD = nh.param<float>("main/parking/regulator_d", 3);
  regulatorP = nh.param<float>("main/parking/regulator_p", 10);
  correctionX = nh.param<float>("main/parking/correction_x", 0.1f);

  safetyDistance = nh.param<float>("main/parking/safety_distance", 0.1f);
  b = nh.param<float>("main/parking/b", 0.32f);
  w = nh.param<float>("main/parking/w", 0.2f);

  // Add a safety distance to the actual car width
  w += safetyDistance;

  r = nh.param<float>("main/parking/minimal_radius", 1);

  r_e = sqrt(r * r + b * b);

  alpha = acos(1 - b * b / (4 * r * r));

  beta = asin(sin(alpha) * r / r_e);

  theta = acos((r - w) / r_e) - beta;

  delta = theta - alpha;

  pdRegler = PIDRegler(nh, regulatorP, regulatorD, velocityForward, 0);

  trajectoryRightWheelPub =
      nh.advertise<nav_msgs::Path>("autu/debug/parking_trajectory", 1);
  trajectoryLeftWheelPub =
      nh.advertise<nav_msgs::Path>("autu/debug/parking_trajectory_odom", 1);
}

void ParkingController::odomCallback(const nav_msgs::OdometryConstPtr &msg) {
  odom = msg;
}

void ParkingController::laserscanCallback(
    const sensor_msgs::LaserScanConstPtr &msg) {
  laserscan = msg;
}

void ParkingController::run() {

  if (odom == nullptr || laserscan == nullptr ||
      !transformListener.canTransform("/base_laser", "/odom", ros::Time(0)))
    return;

  Vector2f vecToCorner;
  tf::Quaternion begin, current;
  switch (state) {
  case DETECT_CORNER:
    vecToCorner = laserUtil.findCorner(laserscan);

    if (vecToCorner[0] == 0 && vecToCorner[1] == 0)
      // No corner detected -> keep searching
      return;

    // convert corner and start position in odom-frame
    try {
      tf::StampedTransform transform;
      transformListener.waitForTransform("base_laser", "/odom", ros::Time(0),
                                         ros::Duration(0.1));
      transformListener.lookupTransform("base_laser", "/odom", ros::Time(0),
                                        transform);

      geometry_msgs::PointStamped cornerInBaseLaser, cornerInOdom;

      // move corner virtually so the car won't hit it
      vecToCorner[1] += safetyDistance;
      cornerInBaseLaser.point.x = vecToCorner[0];
      cornerInBaseLaser.point.y = vecToCorner[1];
      cornerInBaseLaser.header.frame_id = "/base_laser";
      cornerInBaseLaser.header.stamp = ros::Time(0);
      transformListener.transformPoint("/odom", cornerInBaseLaser,
                                       cornerInOdom);
      cornerInOdom.point.z = 0;

      // generate parking-frame in the tf-tree
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

      geometry_msgs::PointStamped startInBaseLaser;

      // convert start position in odom-frame
      startInBaseLaser.point.x = vecToCorner[0] + r * sin(delta);
      startInBaseLaser.point.y = vecToCorner[1] + r * (1 - cos(delta));
      startInBaseLaser.header.frame_id = "/base_laser";
      startInBaseLaser.header.stamp = ros::Time(0);
      transformListener.transformPoint("/odom", startInBaseLaser, start);
      start.point.z = 0;
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }

    publishParkingSpot();
    publishParkingTrajectory();
    state = DRIVE_TO_CORNER;
    break;
  case DRIVE_TO_CORNER:

    try {

      // if odom.x > start.x the start position is reached and the backwards
      // movement can be started
      if (start.point.x - odom->pose.pose.position.x < correctionX)
        state = TURN_RIGHT_INIT;

    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }
    break;
  case TURN_RIGHT_INIT:

    // remember where the turn was started
    curveBegin = odom->pose.pose;

    state = TURN_RIGHT;
    break;
  case TURN_RIGHT:

    tf::quaternionMsgToTF(curveBegin.orientation, begin);
    tf::quaternionMsgToTF(odom->pose.pose.orientation, current);

    // if the yaw angle is bigger than theta, steer left
    if (2 * begin.angle(current) >
        theta) { // 2 * because angle() only returns angle/2
      curveBegin = odom->pose.pose;
      state = TURN_LEFT;
    }

    break;
  case TURN_LEFT:

    tf::quaternionMsgToTF(curveBegin.orientation, begin);
    tf::quaternionMsgToTF(odom->pose.pose.orientation, current);

    if (2 * begin.angle(current) >
        theta) // 2 * because angle() only returns angle/2
      state = STOP;

    break;
  default:
    break;
  }

  pses_basis::Command cmd;
  switch (state) {
  case DRIVE_TO_CORNER:
    // drive to the start position with the pd controller
    pdRegler.drive(odom->pose.pose.position.y - start.point.y, false);
    break;
  case TURN_RIGHT:
    cmd.motor_level = -velocityBackward;
    cmd.steering_level = -maxSteeringRight;
    commandPub.publish(cmd);
    break;
  case TURN_LEFT:
    cmd.motor_level = -velocityBackward;
    cmd.steering_level = maxSteeringLeft;
    commandPub.publish(cmd);
    break;
  case STOP:
    cmd.motor_level = 0;
    // keep steering level from last movement to prevent the servo from making
    // noises
    cmd.steering_level = maxSteeringLeft;
    commandPub.publish(cmd);
    break;
  default:
    break;
  }
}

vector<Vector2f, aligned_allocator<Vector2f>>
ParkingController::calcTrajectory() {
  vector<Vector2f, aligned_allocator<Vector2f>> vectors;
  const Vector2f start(r * sin(delta), r * (1 - cos(delta)));

  const Vector2f v_r(0, r);

  // ICC of the first turn
  const Vector2f startICC = start - v_r;
  vectors.push_back(start);

  // number of vectors used to depict the circles
  const int intervals = 20;

  // sector of circle with angle theta around first ICC
  for (int i = 0; i <= intervals; ++i) {
    Rotation2Df rot(theta * i / intervals);

    vectors.push_back(rot * v_r + startICC);
  }

  // ICC of the second turn
  const Vector2f endICC = Rotation2Df(theta) * (2 * v_r) + startICC;

  // sector of circle with angle theta around second ICC
  for (int i = intervals; i >= 0; --i) {
    Rotation2Df rot(theta * i / intervals);

    vectors.push_back(rot * -v_r + endICC);
  }

  return vectors;
}

void ParkingController::publishParkingTrajectory() {
  nav_msgs::Path msgRightWheel, msgLeftWheel;
  msgRightWheel.header.frame_id = "parking";
  msgLeftWheel.header.frame_id = "parking";

  const auto &vectors = calcTrajectory();

  // add each point of the trajectory to the messages
  for (const Vector2f &vec : vectors) {
    PoseStamped pose;
    pose.pose.position.x = vec[0];
    pose.pose.position.y = vec[1];
    msgRightWheel.poses.push_back(pose);

    // trajectory of the left wheel is nearly the trajectory of the right wheel
    // plus the car-width
    pose.pose.position.y += w;
    msgLeftWheel.poses.push_back(pose);
  }

  trajectoryRightWheelPub.publish(msgRightWheel);
  trajectoryLeftWheelPub.publish(msgLeftWheel);
}
void ParkingController::publishParkingSpot() {
  nav_msgs::Path msgSpot;
  msgSpot.header.frame_id = "parking";
  PoseStamped pose;

  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  msgSpot.poses.push_back(pose);

  pose.pose.position.x = 0;
  pose.pose.position.y = -w;
  msgSpot.poses.push_back(pose);

  pose.pose.position.x = -0.8f;
  pose.pose.position.y = -w;
  msgSpot.poses.push_back(pose);

  pose.pose.position.x = -0.8f;
  pose.pose.position.y = 0;
  msgSpot.poses.push_back(pose);

  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  msgSpot.poses.push_back(pose);

  parkingSpotPub.publish(msgSpot);
}
