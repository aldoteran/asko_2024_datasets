#include "utils/ros_gtsam_utils.h"

namespace ros_gtsam{

// Translates the resulting GTSAM poses into ros msgs, add a timestamp,
// and saves it a rosbag.
void GTSAMResultsToRosbag(const gtsam::Values &results,
                          const std::vector<double> &timestamps){}

// Converts the GTSAM results into ros msgs and appends them to an existing rosbag.
void AppendGTSAMResultsToRosbag(const std::string &path_to_rosbag,
                                const std::string &topic_root,
                                const std::string &global_frame,
                                const std::string &body_frame,
                                const gtsam::Values &results,
                                const std::map<int, double> &timestamps) {
  // Open rosbag in Append mode.
  rosbag::Bag bag;
  bag.open(path_to_rosbag, rosbag::bagmode::Append);

  // The size of the timestamp vector is the number of keyframes.
  size_t size = timestamps.size();

  // Iterate over values.
  for (size_t i = 1; i < size; i++) {
    std::cout << "Writing msg " << i << " of " << size << "\n";
    // GTSAM pose to pose msg in ros.
    const geometry_msgs::PoseWithCovarianceStamped pose_msg =
        GTSAMPose3ToPoseWCovStamped(results.at<gtsam::Pose3>(X(i)), body_frame,
                                    timestamps.at(i));
    // GTSAM pose to TFMessage in ros.
    const tf2_msgs::TFMessage tf_msg =
        GTSAMPose3ToTFMessage(results.at<gtsam::Pose3>(X(i)), global_frame,
                              body_frame, timestamps.at(i));
    // GTSAM pose and vel to odometry msg in ros.
    const nav_msgs::Odometry odom_msg = GTSAMValuesToOdometry(
        results.at<gtsam::Pose3>(X(i)), results.at<gtsam::Vector3>(V(i)),
        global_frame, body_frame, timestamps.at(i));

    std::cout << "Writing message at time " << timestamps.at(i) << "\n";
    bag.write(topic_root + "/pose", ros::Time(timestamps.at(i)), pose_msg);
    bag.write(topic_root + "/odom", ros::Time(timestamps.at(i)), odom_msg);
    bag.write("/tf", ros::Time(timestamps.at(i)), tf_msg);
  }

  bag.close();
}

// gtsam::Pose3 to geometry_msgs::PoseWithCovarianceStamped.
geometry_msgs::PoseWithCovarianceStamped
GTSAMPose3ToPoseWCovStamped(const gtsam::Pose3 &pose, const std::string &frame,
                            double stamp) {
  geometry_msgs::PoseWithCovarianceStamped msg;
  msg.header.stamp = ros::Time(stamp);
  msg.header.frame_id = frame;

  // Orientation.
  const gtsam::Quaternion q = pose.rotation().toQuaternion();
  msg.pose.pose.orientation.x = q.x();
  msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z();
  msg.pose.pose.orientation.w = q.w();
  // Position.
  msg.pose.pose.position.x = pose.x();
  msg.pose.pose.position.y = pose.y();
  msg.pose.pose.position.z = pose.z();

  return msg;
}

nav_msgs::Odometry GTSAMValuesToOdometry(const gtsam::Pose3 &pose,
                                         const gtsam::Vector3 &vel,
                                         const std::string &global_frame,
                                         const std::string &body_frame,
                                         double stamp) {
  nav_msgs::Odometry msg;
  msg.header.stamp = ros::Time(stamp);
  msg.header.frame_id = global_frame;
  msg.child_frame_id = body_frame;

  // Orientation.
  const gtsam::Quaternion q = pose.rotation().toQuaternion();
  msg.pose.pose.orientation.x = q.x();
  msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z();
  msg.pose.pose.orientation.w = q.w();
  // Position.
  msg.pose.pose.position.x = pose.x();
  msg.pose.pose.position.y = pose.y();
  msg.pose.pose.position.z = pose.z();
  // Linear velocity.
  msg.twist.twist.linear.x = vel(0);
  msg.twist.twist.linear.y = vel(1);
  msg.twist.twist.linear.z = vel(2);

  return msg;
}

geometry_msgs::TransformStamped
GTSAMPose3ToTfStamped(const gtsam::Pose3 &pose, const std::string &global_frame,
                      const std::string &body_frame, double stamp) {
  geometry_msgs::TransformStamped msg;
  msg.header.stamp = ros::Time(stamp);
  msg.header.frame_id = global_frame;
  msg.child_frame_id = body_frame;

  // Orientation.
  const gtsam::Quaternion q = pose.rotation().toQuaternion();
  msg.transform.rotation.x = q.x();
  msg.transform.rotation.y = q.y();
  msg.transform.rotation.z = q.z();
  msg.transform.rotation.w = q.w();
  // Position.
  msg.transform.translation.x = pose.x();
  msg.transform.translation.y = pose.y();
  msg.transform.translation.z = pose.z();

  return msg;
}

tf2_msgs::TFMessage GTSAMPose3ToTFMessage(const gtsam::Pose3 &pose,
                                          const std::string &global_frame,
                                          const std::string &body_frame,
                                          double stamp) {
  geometry_msgs::TransformStamped msg;
  msg.header.stamp = ros::Time(stamp);
  msg.header.frame_id = global_frame;
  msg.child_frame_id = body_frame;

  // Orientation.
  const gtsam::Quaternion q = pose.rotation().toQuaternion();
  msg.transform.rotation.x = q.x();
  msg.transform.rotation.y = q.y();
  msg.transform.rotation.z = q.z();
  msg.transform.rotation.w = q.w();
  // Position.
  msg.transform.translation.x = pose.x();
  msg.transform.translation.y = pose.y();
  msg.transform.translation.z = pose.z();

  tf2_msgs::TFMessage tf_msg;
  tf_msg.transforms.push_back(msg);
  return tf_msg;
}

} // namespace ros_gtsam.
