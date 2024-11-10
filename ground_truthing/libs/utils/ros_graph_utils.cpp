
#include "ros_graph_utils.h"

// Converts ROS Odometry msg into a 3DPoseMeasurement struct.
Pose3dMeasurement Pose3dMeasFromOdometry(const nav_msgs::Odometry &msg) {

  Pose3dMeasurement meas;

  meas.stamp = msg.header.stamp.toSec();
  meas.ref_frame = msg.header.frame_id;
  meas.frame = msg.child_frame_id;

  meas.pose = gtsam::Pose3(EigenAffineFromOdom(msg).matrix());
  meas.noise = gtsam::noiseModel::Gaussian::Covariance(
      Eigen6x6PoseCovFromPoseCov(msg.pose));

  return meas;
}

// Converts ROS PoseWithCovarianceStamped message into a
// 3DPoseBetweenMeasruement struct.
Pose3dMeasurement Pose3dMeasFromPoseCovStamped(
    const geometry_msgs::PoseWithCovarianceStamped &msg) {

  Pose3dMeasurement meas;

  meas.stamp = msg.header.stamp.toSec();
  meas.ref_frame = msg.header.frame_id;

  meas.pose = gtsam::Pose3(EigenAffineFromPoseCovStamped(msg).matrix());
  meas.noise =
      gtsam::noiseModel::Gaussian::Covariance(Eigen6x6PoseCovFromPoseCov(msg.pose));

  return meas;
}

// Converts ROS Marker msg into a 3dRangeBearingMeasurement.
RangeBearing3dMeasurement
RangeBearing3dFromMarker(const visualization_msgs::Marker &msg) {

  RangeBearing3dMeasurement meas;

  meas.stamp = msg.header.stamp.toSec();
  meas.ref_frame = msg.header.frame_id;

  const gtsam::Point3 position = EigenPositionFromMarker(msg);
  meas.range = position.norm();
  meas.bearing = gtsam::Unit3::FromPoint3(position);

  return meas;
}

