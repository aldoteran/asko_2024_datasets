/*
 * @file ros_graph_utils.h
 * @brief Helper functions to deal with ROS message conversions.
 */

#ifndef UTILS_ROS_GRAPH_UTILS_H_
#define UTILS_ROS_GRAPH_UTILS_H_

#include "ros_eigen_conversions.h"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <string>

struct Pose3dMeasurement {
  double stamp;
  gtsam::Pose3 pose;
  gtsam::noiseModel::Gaussian::shared_ptr noise;
  std::string frame;
  std::string ref_frame;
};

struct RangeBearing3dMeasurement {
  double stamp;
  double range;
  gtsam::Unit3 bearing;
  gtsam::noiseModel::Gaussian::shared_ptr noise;
  std::string frame;
  std::string ref_frame;
};

/*
 * Add new measurement structs here.
 */

// Converts ROS Odometry msg into a 3DPoseMeasurement struct.
//struct Pose3dMeasurement Pose3dMeasFromOdometry(const nav_msgs::Odometry &msg);

// Converts ROS PoseWithCovarianceStamped message into a
// 3DPoseBetweenMeasruement struct.
//struct Pose3dMeasurement Pose3dMeasFromPoseCovStamped(
    //const geometry_msgs::PoseWithCovarianceStamped &pose_msg);

// Converts ROS Marker msg into a 3dRangeBearingMeasurement.
//struct RangeBearing3dMeasurement RangeBearing3dMeasFromMarker(
    //const visualization_msgs::Marker &msg);

/*
 * Add more conversion functions here.
 */

#endif  // UTILS_ROS_GRAPH_UTILS_H_
