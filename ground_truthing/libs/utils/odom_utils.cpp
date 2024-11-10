#include "odom_utils.h"

void BroadcastTFPose(const geometry_msgs::Pose& pose, const ros::Time& stamp,
                     const std::string& map_frame_id, const std::string& link_frame_id) {
  // Broadcast the tf from map_frame -> link_frame_id.
  static tf::TransformBroadcaster br;

  tf::Transform transform(
      tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z,
                     pose.orientation.w),
      tf::Vector3(pose.position.x, pose.position.y, pose.position.z));

  br.sendTransform(
      tf::StampedTransform(transform, stamp, map_frame_id, link_frame_id));
}

geometry_msgs::PoseStamped EigenTransformToPoseMsg(const Eigen::Affine3d& tfm) {
  geometry_msgs::PoseStamped pose;
  tf::poseEigenToMsg(tfm, pose.pose);
  return pose;
}

