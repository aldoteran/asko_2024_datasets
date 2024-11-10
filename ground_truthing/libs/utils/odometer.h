/**
 * @file odometer.h
 * @brief Main class to deal with the AUV's odometry measurements.
 */

#ifndef ODOMETER_H_
#define ODOMETER_H_

#include <Eigen/Core>

#include <ros/time.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>
#include <vector>

#include <ros/publisher.h>
#include <sensor_msgs/Imu.h>
#include <ixblue_ins_msgs/Ins.h>
#include <nav_msgs/Odometry.h>

#define PI 3.15159

class Odometer {
 public:

  // Constructor requires the node handle to initialize internal publishers.
  Odometer(ros::NodeHandle& nh);

  void InsCallback(const ixblue_ins_msgs::Ins& msg);
  void ImuCallback(const sensor_msgs::Imu& msg);

  // Publisher for the local relative odometry measurement.
  ros::Publisher odom_pub;

  // Publisher for the ENU-referenced accumulated odometry (~dead reckoning).
  ros::Publisher map_odom_pub;

  // Calculates the relative pose between last_pose_ and the enu_odom
  // pose, appends the velocity and covariance, and publishes the odom msg.
  nav_msgs::Odometry ComputeRelative(const nav_msgs::Odometry& enu_odom);

  nav_msgs::Odometry ComposeOdometryMsg(const geometry_msgs::TransformStamped& tfm);

  bool is_imu_init = false;
  bool is_ins_init = false;

 private:

  double heading_;
  double pitch_;
  double roll_;
  std::vector<double> speed_vessel_frame_;
  std::vector<double> angular_velocity_;
  std::vector<double> position_covariance_;
  std::vector<double> attitude_covariance_;
  std::vector<double> speed_vessel_frame_covariance_;
  std::vector<double> angular_velocity_covariance_;

  // Most recent AUV pose.
  geometry_msgs::Pose last_pose_;
  nav_msgs::Odometry last_odom_;

  // Flag to check whether there's a previous pose.
  bool is_init_ = false;

};

#endif  // ODOMETER_H_
