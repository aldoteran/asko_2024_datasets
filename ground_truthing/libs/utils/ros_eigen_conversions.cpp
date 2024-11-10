/**
 * @file ros_eigen_conversions.cpp
 * @brief ROS utilities to bridge Eigen and ROS.
 */

#include "ros_eigen_conversions.h"

namespace dockslam{

Eigen::Vector3d EigenVelFromOdom(const nav_msgs::Odometry &msg) {
  return Eigen::Vector3d{msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                         msg.twist.twist.linear.z};
}

Eigen::Vector3d EigenPositionFromPoseCov(
    const geometry_msgs::PoseWithCovariance &msg) {
  return Eigen::Vector3d{msg.pose.position.x, msg.pose.position.y,
                         msg.pose.position.z};
}

Eigen::Quaterniond EigenQuatFromPoseCov(
    const geometry_msgs::PoseWithCovariance &msg) {
  return Eigen::Quaterniond{
      msg.pose.orientation.w, msg.pose.orientation.x,
      msg.pose.orientation.y, msg.pose.orientation.z};
}

Eigen::Matrix<double, 6, 6> Eigen6x6PoseCovFromPoseCov(
    const geometry_msgs::PoseWithCovariance &msg) {
  // FIXME: Should be able to do it smarter.
  Eigen::Matrix<double, 6, 6> covariance;
  int i = 0;
  for (int j = 0; j < 6; j++) {
    for (int k = 0; k < 6; k++) {
      covariance(j, k) = msg.covariance[i];
      i++;
    }
  }
  return covariance;
}

Eigen::Matrix<double, 3, 3> Eigen3x3VelCovFromOdom(
        const nav_msgs::Odometry &msg) {
    // Twist covariance from msg includes angular velocity as well.
  // FIXME: Should be able to do it smarter.
    Eigen::Matrix<double, 6, 6> covariance;
    int i = 0;
    for (int j = 0; j < 6; j++) {
      for (int k = 0; k < 6; k++) {
        covariance(j, k) = msg.twist.covariance[i];
        i++;
      }
    }
    // Extract 3 by 3 block containing linear velocity covariance.
    return covariance.block(0, 0, 3, 3);
}

Eigen::Affine3d EigenAffineFromOdom(const nav_msgs::Odometry &msg){
    Eigen::Affine3d pose;
    pose.linear() = EigenQuatFromPoseCov(msg.pose).toRotationMatrix();
    pose.translation() = EigenPositionFromPoseCov(msg.pose);

    return pose;
}

Eigen::Affine3d EigenAffineFromPoseCovStamped(
        const geometry_msgs::PoseWithCovarianceStamped& msg){
    Eigen::Affine3d pose;
    pose.linear() = EigenQuatFromPoseCov(msg.pose).toRotationMatrix();
    pose.translation() = EigenPositionFromPoseCov(msg.pose);

    return pose;
}

Eigen::Vector3d EigenPositionFromMarker(const visualization_msgs::Marker &msg) {
  return Eigen::Vector3d{msg.pose.position.x, msg.pose.position.y,
                         msg.pose.position.z};
}

Eigen::Vector3d EigenTranslationFromTransformStamped(
        const geometry_msgs::TransformStamped &msg) {
  return Eigen::Vector3d{msg.transform.translation.x,
                         msg.transform.translation.y,
                         msg.transform.translation.z};
}

Eigen::Affine3d EigenAffineFromTransformStamped(
        const geometry_msgs::TransformStamped &msg) {
  const Eigen::Vector3d translation = {msg.transform.translation.x,
                                       msg.transform.translation.y,
                                       msg.transform.translation.z};

  const Eigen::Quaternion<double, Eigen::DontAlign> rotation = {
      msg.transform.rotation.w, msg.transform.rotation.x,
      msg.transform.rotation.y, msg.transform.rotation.z};

  Eigen::Affine3d eigen_tfm;
  eigen_tfm.linear() = rotation.toRotationMatrix();
  eigen_tfm.translation() = translation;

  return eigen_tfm;
}

Eigen::MatrixXd GtsamPoseCovarianceFromRos(const Eigen::MatrixXd& ros_covariance){
    Eigen::Matrix<double, 6, 6> gtsam_covariance;

    // Top left 3x3 block is for orientation on GTSAM.
    gtsam_covariance.block(0, 0, 3, 3) = ros_covariance.block(3, 3, 3, 3);
    // Bottom right 3x3 block is for translation on GTSAM.
    gtsam_covariance.block(3, 3, 3, 3) = ros_covariance.block(0, 0, 3, 3);
    // Switch off-diagonals.
    gtsam_covariance.block(0, 3, 3, 3) = ros_covariance.block(3, 0, 3, 3);
    gtsam_covariance.block(3, 0, 3, 3) = ros_covariance.block(0, 3, 3, 3);

    return gtsam_covariance;
}

Eigen::MatrixXd RosPoseCovarianceFromGtsam(
        const Eigen::MatrixXd &gtsam_covariance) {
    Eigen::Matrix<double, 6, 6> ros_covariance;

    // Top left 3x3 block is for translation on ROS.
    ros_covariance.block(0, 0, 3, 3) = gtsam_covariance.block(3, 3, 3, 3);
    // Bottom right 3x3 block is for orientation on ROS.
    ros_covariance.block(3, 3, 3, 3) = gtsam_covariance.block(0, 0, 3, 3);
    // Switch off-diagonals.
    ros_covariance.block(0, 3, 3, 3) = gtsam_covariance.block(3, 0, 3, 3);
    ros_covariance.block(3, 0, 3, 3) = gtsam_covariance.block(0, 3, 3, 3);

    return ros_covariance;
}

} // namespace dockslam.
