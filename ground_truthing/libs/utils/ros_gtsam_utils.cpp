#include "utils/ros_gtsam_utils.h"

namespace ros_gtsam{

// Translates the resulting GTSAM poses into ros msgs, add a timestamp,
// and saves it a rosbag.
void GTSAMResultsToRosbag(const gtsam::Values &results,
                          const std::vector<double> &timestamps){}

// Converts the GTSAM results into ros msgs and appends them to an existing rosbag.
void AppendGTSAMResultsToRosbag(const std::string &path_to_rosbag,
                                const gtsam::Values &results,
                                const std::vector<double> &timestamps){
    // Open rosbag in Append mode.
    rosbag::Bag bag;
    bag.open(path_to_rosbag, rosbag::bagmode::Append);

    // The size of the timestamp vector is the number of keyframes.
    size_t size = timestamps.size();

    // Iterate over
    for (size_t i = 0; i < size; i++){

    }
}

// gtsam::Pose3 to geometry_msgs::PoseWithCovarianceStamped.
geometry_msgs::PoseWithCovarianceStamped
GTSAMPose3ToPoseWCovStamped(const gtsam::Pose3 &pose, const std::string &frame,
        double stamp) {
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header.stamp = ros.Time(stamp);
    msg.header.frame = frame;

    // Orientation.
    const gtsam::Quatenion q = pose.rotation().toQuaterion();
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

} // namespace ros_gtsam.
