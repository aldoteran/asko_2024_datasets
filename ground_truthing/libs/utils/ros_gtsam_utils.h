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

#include "utils/ros_eigen_conversions.h"
#include "utils/tfm_utils.h"

namespace ros_gtsam{

// Translates the resulting GTSAM poses into ros msgs, add a timestamp,
// and saves it a rosbag.
void GTSAMResultsToRosbag(const gtsam::Values &results,
                          const std::vector<double> &timestamps);

// Converts the GTSAM results into ros msgs and appends them to an existing rosbag.
void AppendGTSAMResultsToRosbag(const std::string &path_to_rosbag,
                                const gtsam::Values &results,
                                const std::vector<double> &timestamps);

// gtsam::Pose3 to geometry_msgs::PoseWithCovarianceStamped.
// TODO: Add covariance as well.
geometry_msgs::PoseWithCovarianceStamped
GTSAMPose3ToPoseWCovStamped(const gtsam::Pose3 &pose, const std::string &frame,
                            double stamp);

} // namespace ros_gtsam

#endif // _UTILS_ROS_GTSAM_UTILS_H_
