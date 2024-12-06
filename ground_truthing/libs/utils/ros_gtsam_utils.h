/**
 * @file ros_gtsam_utils.h
 * @brief Usefult tools for working with GTSAM and ROS.
 * @date November 5, 2024
 * @author Aldo Teran Espinoza
 * @author_email aldot@kth.se
 */
#ifndef _UTILS_ROS_GTSAM_UTILS_H_
#define _UTILS_ROS_GTSAM_UTILS_H_

#include <iostream>
#include <memory>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Quaternion.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_msgs/TFMessage.h>

#include "utils/ros_eigen_conversions.h"
#include "utils/tfm_utils.h"

using gtsam::symbol_shorthand::X; // Pose.
using gtsam::symbol_shorthand::V; // Velocity.

namespace ros_gtsam{

// Translates the resulting GTSAM poses into ros msgs, add a timestamp,
// and saves it a rosbag.
void GTSAMResultsToRosbag(const gtsam::Values &results,
                          const std::vector<double> &timestamps);

// Converts the GTSAM results into ros msgs and appends them to an existing rosbag.
void AppendGTSAMResultsToRosbag(const std::string &path_to_rosbag,
                                const std::string &topic_root,
                                const std::string &global_frame,
                                const std::string &body_frame,
                                const gtsam::Values &results,
                                const std::map<int, double> &timestamps);

// gtsam::Pose3 to geometry_msgs::PoseWithCovarianceStamped.
// TODO: Add covariance as well.
geometry_msgs::PoseWithCovarianceStamped
GTSAMPose3ToPoseWCovStamped(const gtsam::Pose3 &pose, const std::string &frame,
                            double stamp);

// GTSAM pose and velocity values into an Odometry rosmsg.
// TODO: Add covariance as well.
nav_msgs::Odometry GTSAMValuesToOdometry(const gtsam::Pose3 &pose,
                                         const gtsam::Vector3 &vel,
                                         const std::string &global_frame,
                                         const std::string &body_frame,
                                         double stamp);

// GTSAM Pose3 to ros TransformStamped.
geometry_msgs::TransformStamped
GTSAMPose3ToTfStamped(const gtsam::Pose3 &pose, const std::string &global_frame,
                      const std::string &body_frame, double stamp);

// GTSAM Pose3 to tf2 TFMessage.
tf2_msgs::TFMessage GTSAMPose3ToTFMessage(const gtsam::Pose3 &pose,
                                          const std::string &global_frame,
                                          const std::string &body_frame,
                                          double stamp);
} // namespace ros_gtsam

#endif // _UTILS_ROS_GTSAM_UTILS_H_
