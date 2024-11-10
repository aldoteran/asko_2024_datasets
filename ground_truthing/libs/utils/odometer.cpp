#include "odometer.h"

double constrainAngle(double x) {
  x = fmod(x + 180, 360);
  if (x < 0)
    x += 360;
  return x - 180;
}

Odometer::Odometer(ros::NodeHandle& nh) {

  // Initialize all publishers.
  odom_pub =
      nh.advertise<nav_msgs::Odometry>("/lolo/dr/relative/odom", 1);
  map_odom_pub =
      nh.advertise<nav_msgs::Odometry>("lolo/dr/odom", 1);

}

void Odometer::InsCallback(const ixblue_ins_msgs::Ins &msg) {
  position_covariance_ = std::vector<double>{
      msg.position_covariance[0], msg.position_covariance[1],
      msg.position_covariance[2], msg.position_covariance[3],
      msg.position_covariance[4], msg.position_covariance[5],
      msg.position_covariance[6], msg.position_covariance[7],
      msg.position_covariance[8]};

  heading_ = constrainAngle(msg.heading) * PI / 180.0;
  pitch_ = msg.pitch * PI / 180.0;
  roll_ = msg.roll * PI / 180.0;

  attitude_covariance_ = std::vector<double>{
      msg.attitude_covariance[0], msg.attitude_covariance[1],
      msg.attitude_covariance[2], msg.attitude_covariance[3],
      msg.attitude_covariance[4], msg.attitude_covariance[5],
      msg.attitude_covariance[6], msg.attitude_covariance[7],
      msg.attitude_covariance[8]};

  speed_vessel_frame_ =
      std::vector<double>{msg.speed_vessel_frame.x, msg.speed_vessel_frame.y,
                          msg.speed_vessel_frame.z};

  speed_vessel_frame_covariance_ =
      std::vector<double>{msg.speed_vessel_frame_covariance[0],
                          msg.speed_vessel_frame_covariance[1],
                          msg.speed_vessel_frame_covariance[2],
                          msg.speed_vessel_frame_covariance[3],
                          msg.speed_vessel_frame_covariance[4],
                          msg.speed_vessel_frame_covariance[5],
                          msg.speed_vessel_frame_covariance[6],
                          msg.speed_vessel_frame_covariance[7],
                          msg.speed_vessel_frame_covariance[8]};
  if (!is_ins_init) {
    is_ins_init = true;
  }
}

void Odometer::ImuCallback(const sensor_msgs::Imu &msg) {
  angular_velocity_ = std::vector<double>{
      msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z};
  angular_velocity_covariance_ = std::vector<double>{
      msg.angular_velocity_covariance[0], msg.angular_velocity_covariance[1],
      msg.angular_velocity_covariance[2], msg.angular_velocity_covariance[3],
      msg.angular_velocity_covariance[4], msg.angular_velocity_covariance[5],
      msg.angular_velocity_covariance[6], msg.angular_velocity_covariance[7],
      msg.angular_velocity_covariance[8]};

  if (!is_imu_init) {
    is_imu_init = true;
  }

}

nav_msgs::Odometry
Odometer::ComposeOdometryMsg(const geometry_msgs::TransformStamped &tfm) {
  nav_msgs::Odometry odom;
  odom.header = tfm.header;
  odom.child_frame_id = tfm.child_frame_id;
  odom.pose.pose.position.x = tfm.transform.translation.x;
  odom.pose.pose.position.y = tfm.transform.translation.y;
  odom.pose.pose.position.z = tfm.transform.translation.z;
  odom.pose.pose.orientation = tfm.transform.rotation;
  odom.pose.covariance[0] = position_covariance_[0];
  odom.pose.covariance[1] = position_covariance_[1];
  odom.pose.covariance[2] = position_covariance_[2];
  odom.pose.covariance[3] = position_covariance_[3];
  odom.pose.covariance[4] = position_covariance_[4];
  odom.pose.covariance[5] = position_covariance_[5];
  odom.pose.covariance[6] = position_covariance_[6];
  odom.pose.covariance[7] = position_covariance_[7];
  odom.pose.covariance[8] = position_covariance_[8];
  odom.pose.covariance[27] = attitude_covariance_[0];
  odom.pose.covariance[28] = attitude_covariance_[1];
  odom.pose.covariance[29] = attitude_covariance_[2];
  odom.pose.covariance[30] = attitude_covariance_[3];
  odom.pose.covariance[31] = attitude_covariance_[4];
  odom.pose.covariance[32] = attitude_covariance_[5];
  odom.pose.covariance[33] = attitude_covariance_[6];
  odom.pose.covariance[34] = attitude_covariance_[7];
  odom.pose.covariance[35] = attitude_covariance_[8];
  odom.twist.twist.linear.x = speed_vessel_frame_[0];
  odom.twist.twist.linear.y = speed_vessel_frame_[1];
  odom.twist.twist.linear.z = speed_vessel_frame_[2];
  odom.twist.twist.angular.x = angular_velocity_[0];
  odom.twist.twist.angular.y = angular_velocity_[1];
  odom.twist.twist.angular.z = angular_velocity_[2];
  odom.twist.covariance[0] = speed_vessel_frame_covariance_[0];
  odom.twist.covariance[1] = speed_vessel_frame_covariance_[1];
  odom.twist.covariance[2] = speed_vessel_frame_covariance_[2];
  odom.twist.covariance[3] = speed_vessel_frame_covariance_[3];
  odom.twist.covariance[4] = speed_vessel_frame_covariance_[4];
  odom.twist.covariance[5] = speed_vessel_frame_covariance_[5];
  odom.twist.covariance[6] = speed_vessel_frame_covariance_[6];
  odom.twist.covariance[7] = speed_vessel_frame_covariance_[7];
  odom.twist.covariance[8] = speed_vessel_frame_covariance_[8];
  odom.twist.covariance[27] = angular_velocity_covariance_[0];
  odom.twist.covariance[28] = angular_velocity_covariance_[1];
  odom.twist.covariance[29] = angular_velocity_covariance_[2];
  odom.twist.covariance[30] = angular_velocity_covariance_[3];
  odom.twist.covariance[31] = angular_velocity_covariance_[4];
  odom.twist.covariance[32] = angular_velocity_covariance_[5];
  odom.twist.covariance[33] = angular_velocity_covariance_[6];
  odom.twist.covariance[34] = angular_velocity_covariance_[7];
  odom.twist.covariance[35] = angular_velocity_covariance_[8];

  if (!is_init_){
      last_odom_ = odom;
      is_init_ = true;
  }

  return odom;
}

nav_msgs::Odometry Odometer::ComputeRelative(const nav_msgs::Odometry& odom) {
  // Let's put everything on Eigen first to make it easy to work with.
  Eigen::Affine3d previous_pose;
  Eigen::Affine3d current_pose;
  Eigen::Affine3d relative_pose;


  tf::poseMsgToEigen(last_odom_.pose.pose, previous_pose);
  tf::poseMsgToEigen(odom.pose.pose, current_pose);

  const Eigen::Matrix3d previous_rot = previous_pose.rotation();
  const Eigen::Matrix3d current_rot = current_pose.rotation();
  const Eigen::Vector3d previous_trans = previous_pose.translation();
  const Eigen::Vector3d current_trans = current_pose.translation();

  // Now we want to calculate the transformation from the previous pose
  // to the current pose in the AUV's frame: ^(A_t-1)_(A_t) T.
  relative_pose.linear() = previous_rot.transpose() * current_rot;
  relative_pose.translation() = previous_rot.transpose() *
                                (current_trans - previous_trans);

  // Now we build the Odometry message: Header.
  nav_msgs::Odometry rel_odom;
  rel_odom.header.stamp = odom.header.stamp;
  rel_odom.header.frame_id = odom.child_frame_id;
  rel_odom.child_frame_id = odom.child_frame_id;

  // Pose w/ covariance.
  tf::poseEigenToMsg(relative_pose, rel_odom.pose.pose);
  //tf::transformEigenToMsg(relative_pose, relative_tfm.transform);
  tf2::Transform relative_tfm;
  tf2::fromMsg(rel_odom.pose.pose, relative_tfm);
  rel_odom.pose.covariance =
      tf2::transformCovariance(odom.pose.covariance, relative_tfm);

  // Twist w/ covariance.
  rel_odom.twist = odom.twist;

   //Replace last pose for latest pose.
  last_odom_ = odom;

  return rel_odom;
}

