/**
 * @file ros_eigen_conversions.h
 * @brief ROS utilities to bridge Eigen and ROS.
 */

#ifndef UTILS_ROS_EIGEN_CONVERSIONS_H_
#define UTILS_ROS_EIGEN_CONVERSIONS_H_

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>

#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

namespace dockslam{

// Extract the linear velocity from the odometry msg into an Eigen Vector3d.
Eigen::Vector3d EigenVelFromOdom(const nav_msgs::Odometry& msg);

// Extract the position vector from the odometry msg into an Eigen Vector3d.
Eigen::Vector3d EigenPositionFromPoseCov(const geometry_msgs::PoseWithCovariance& msg);

// Extract the quaternion orientation from the odometry msg into an Eigen
// Quaterniond.
Eigen::Quaterniond EigenQuatFromPoseCov(
    const geometry_msgs::PoseWithCovariance& msg);

// Extract the 6x6 pose covariance matrix from the odometry msg into an
// Eigen 6x6 matrix.
Eigen::Matrix<double, 6, 6> Eigen6x6PoseCovFromPoseCov(
    const geometry_msgs::PoseWithCovariance& msg);

// Extract the 3x3 velocity covariance matrix from the odometry msg into
// an Eigen 3x3 matrix.
Eigen::Matrix<double, 3, 3> Eigen3x3VelCovFromOdom(
    const nav_msgs::Odometry& msg);

// Extract the 6DOF pose from the odometry msg into an Eigen Affine3d.
Eigen::Affine3d EigenAffineFromOdom(const nav_msgs::Odometry& msg);

// Extract the 6DOF pose from a PoseWithCovarianceStamped into an Affine3d.
Eigen::Affine3d EigenAffineFromPoseCovStamped(
    const geometry_msgs::PoseWithCovarianceStamped& msg);

// Extract the position vector from a Maker msg to an Eigen Vector3d.
Eigen::Vector3d EigenPositionFromMarker(const visualization_msgs::Marker& msg);

Eigen::Vector3d EigenTranslationFromTransformStamped(
    const geometry_msgs::TransformStamped& msg);

Eigen::Affine3d EigenAffineFromTransformStamped(
    const geometry_msgs::TransformStamped& msg);

Eigen::MatrixXd GtsamPoseCovarianceFromRos(
    const Eigen::MatrixXd& ros_covariance);

Eigen::MatrixXd RosPoseCovarianceFromGtsam(
    const Eigen::MatrixXd& gtsam_covariance);

} // namespace dockslam.

#endif  // UTILS_ROS_EIGEN_CONVERSIONS_H_
