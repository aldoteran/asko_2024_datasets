/**
 * @file odom_utils.h
 * @brief Helper functions to deal with odom measurements.
 */

#ifndef ODOM_UTILS_H_
#define ODOM_UTILS_H_

#include <string>

#include <Eigen/Core>

#include <ros/time.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <eigen_conversions/eigen_msg.h>

// Broadcast the TF of the pose from map_frame_id to link_frame_id at time stamp.
void BroadcastTFPose(const geometry_msgs::Pose& pose, const ros::Time& stamp,
                     const std::string& map_frame_id, const std::string& link_frame_id);

geometry_msgs::PoseStamped EigenTransformToPoseMsg(const Eigen::Affine3d& tfm);

#endif  // ODOM_UTILS_H_
