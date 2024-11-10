/* ----------------------------------------------------------------------------

 * Underwater Fun Research Co.,
 * Seattle - Stockholm - Örebro
 * Authors: Aldo Teran Espinoza, Antonio Teran Espinoza, David Baxter.

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file main.cpp
 * @brief Read rosbag, build pose graph with target's data, save graph.
 * @date November 5, 2024
 * @author Aldo Teran Espinoza
 * @author_email aldot@kth.se
 */
#include <iostream>
#include <memory>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <sbg_driver/SbgImuData.h>
#include <sbg_driver/SbgGpsHdt.h>
#include <sbg_driver/SbgGpsPos.h>

#include <GeographicLib/UTMUPS.hpp>

#include "utils/ros_eigen_conversions.h"
#include "backend/graph_manager.h"

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

int main(int argc, char *argv[]){

  ros::init(argc, argv, "bag_to_graph");
  ros::NodeHandle nh_;

  // Initialize graph here with a config file from the input aguments.
  dockslam::GraphManager::Config graph_config;
  // Priors.
  nh_.getParam("prior_roll_stddev", graph_config.target_init_pose_stddev(0));
  nh_.getParam("prior_pitch_stddev", graph_config.target_init_pose_stddev(1));
  nh_.getParam("prior_yaw_stddev", graph_config.target_init_pose_stddev(2));
  nh_.getParam("prior_x_stddev", graph_config.target_init_pose_stddev(3));
  nh_.getParam("prior_y_stddev", graph_config.target_init_pose_stddev(4));
  nh_.getParam("prior_z_stddev", graph_config.target_init_pose_stddev(5));
  nh_.getParam("target_init_vel_x_stddev",
               graph_config.target_init_vel_stddev(0));
  nh_.getParam("target_init_vel_y_stddev",
               graph_config.target_init_vel_stddev(1));
  nh_.getParam("target_init_vel_z_stddev",
               graph_config.target_init_vel_stddev(2));
  // Previously estimated IMU biases.
  nh_.getParam("init_accel_bias_x", graph_config.init_accel_bias(0));
  nh_.getParam("init_accel_bias_y", graph_config.init_accel_bias(1));
  nh_.getParam("init_accel_bias_z", graph_config.init_accel_bias(2));
  nh_.getParam("init_omega_bias_x", graph_config.init_omega_bias(0));
  nh_.getParam("init_omega_bias_y", graph_config.init_omega_bias(1));
  nh_.getParam("init_omega_bias_z", graph_config.init_omega_bias(2));
  // Measurement noises. USBL:
  nh_.getParam("usbl_x_stddev", graph_config.usbl_noise_stddev(0));
  nh_.getParam("usbl_y_stddev", graph_config.usbl_noise_stddev(1));
  nh_.getParam("usbl_z_stddev", graph_config.usbl_noise_stddev(2));
  // IMU noise model parameters.
  nh_.getParam("accel_noise_x", graph_config.imu_accel_noise_stddev(0));
  nh_.getParam("accel_noise_y", graph_config.imu_accel_noise_stddev(1));
  nh_.getParam("accel_noise_z", graph_config.imu_accel_noise_stddev(2));
  nh_.getParam("accel_bias_x", graph_config.imu_accel_bias_stddev(0));
  nh_.getParam("accel_bias_y", graph_config.imu_accel_bias_stddev(1));
  nh_.getParam("accel_bias_z", graph_config.imu_accel_bias_stddev(2));
  nh_.getParam("omega_noise_x", graph_config.imu_omega_noise_stddev(0));
  nh_.getParam("omega_noise_y", graph_config.imu_omega_noise_stddev(1));
  nh_.getParam("omega_noise_z", graph_config.imu_omega_noise_stddev(2));
  nh_.getParam("omega_bias_x", graph_config.imu_omega_bias_stddev(0));
  nh_.getParam("omega_bias_y", graph_config.imu_omega_bias_stddev(1));
  nh_.getParam("omega_bias_z", graph_config.imu_omega_bias_stddev(2));
  nh_.getParam("imu_frequency", graph_config.imu_frequency);
  // Extrinsic calibrations.
  nh_.getParam("camera_extrinsics_roll", graph_config.chaser_camera_extrinsics(0));
  nh_.getParam("camera_extrinsics_pitch", graph_config.chaser_camera_extrinsics(1));
  nh_.getParam("camera_extrinsics_yaw", graph_config.chaser_camera_extrinsics(2));
  nh_.getParam("camera_extrinsics_x", graph_config.chaser_camera_extrinsics(3));
  nh_.getParam("camera_extrinsics_y", graph_config.chaser_camera_extrinsics(4));
  nh_.getParam("camera_extrinsics_z", graph_config.chaser_camera_extrinsics(5));
  nh_.getParam("ch_usbl_extrinsics_roll", graph_config.chaser_usbl_extrinsics(0));
  nh_.getParam("ch_usbl_extrinsics_pitch", graph_config.chaser_usbl_extrinsics(1));
  nh_.getParam("ch_usbl_extrinsics_yaw", graph_config.chaser_usbl_extrinsics(2));
  nh_.getParam("ch_usbl_extrinsics_x", graph_config.chaser_usbl_extrinsics(3));
  nh_.getParam("ch_usbl_extrinsics_y", graph_config.chaser_usbl_extrinsics(4));
  nh_.getParam("ch_usbl_extrinsics_z", graph_config.chaser_usbl_extrinsics(5));
  nh_.getParam("fiducials_extrinsics_roll", graph_config.target_fiducials_extrinsics(0));
  nh_.getParam("fiducials_extrinsics_pitch", graph_config.target_fiducials_extrinsics(1));
  nh_.getParam("fiducials_extrinsics_yaw", graph_config.target_fiducials_extrinsics(2));
  nh_.getParam("fiducials_extrinsics_x", graph_config.target_fiducials_extrinsics(3));
  nh_.getParam("fiducials_extrinsics_y", graph_config.target_fiducials_extrinsics(4));
  nh_.getParam("fiducials_extrinsics_z", graph_config.target_fiducials_extrinsics(5));
  nh_.getParam("tgt_usbl_extrinsics_roll", graph_config.target_usbl_extrinsics(0));
  nh_.getParam("tgt_usbl_extrinsics_pitch", graph_config.target_usbl_extrinsics(1));
  nh_.getParam("tgt_usbl_extrinsics_yaw", graph_config.target_usbl_extrinsics(2));
  nh_.getParam("tgt_usbl_extrinsics_x", graph_config.target_usbl_extrinsics(3));
  nh_.getParam("tgt_usbl_extrinsics_y", graph_config.target_usbl_extrinsics(4));
  nh_.getParam("tgt_usbl_extrinsics_z", graph_config.target_usbl_extrinsics(5));


  std::unique_ptr<dockslam::GraphManager> gm_ = nullptr;
  gm_ = std::make_unique<dockslam::GraphManager>(graph_config);

  // Read bag.
  std::string bagfile;
  nh_.getParam("path_to_bag", bagfile);
  rosbag::Bag bag;
  std::cout << "Reading bag: " << bagfile << std::endl;
  bag.open(bagfile, rosbag::bagmode::Read);

  // Add topics we want to read into a topics list and query the topics.
  const std::string chaser_global_odom_topic = "/lolo/dr/odom";
  const std::string chaser_optical_topic = "/lolo/perception/optical_pose";
  const std::string target_global_odom_topic = "/service_boat/dr/odom";
  const std::string target_usbl_topic = "/service_boat/enu/usbl_fix";
  const std::string target_imu_topic = "/sbg/imu_data";
  const std::string target_gps_topic = "/sbg/gps_pos";
  const std::string target_hdt_topic = "/sbg/gps_hdt";

  std::vector<std::string> topics;
  topics.push_back(chaser_global_odom_topic);
  topics.push_back(chaser_optical_topic);
  topics.push_back(target_global_odom_topic);
  topics.push_back(target_usbl_topic);
  topics.push_back(target_imu_topic);
  topics.push_back(target_gps_topic);
  topics.push_back(target_hdt_topic);
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  // (debug)
  std::cout << "Processing " << view.size() << " messages." << std::endl;
  int ch_global_odom_count = 0;
  int ch_optical_count = 0;
  int tgt_usbl_count = 0;
  int tgt_imu_count = 0;
  int tgt_gps_count = 0;
  int tgt_hdt_count = 0;

  // Constant UTM offset. What's the best way to do this?
  double UTM_X = 651000.5;
  double UTM_Y = 6523400.5;

  bool is_first_target_odom = true;
  size_t i = 0;
  // Main for loop. Here we'll get all the data and build the graph.
  BOOST_FOREACH (rosbag::MessageInstance const m, view) {
    if (i > 1){ break; }

    // ------- target global navigation ---------
    if ((m.getTopic() == target_global_odom_topic) ||
        ("/" + m.getTopic() == target_global_odom_topic)) {
      nav_msgs::Odometry::ConstPtr global_odom_msg =
          m.instantiate<nav_msgs::Odometry>();
      if (global_odom_msg != nullptr) {
        ch_global_odom_count++;

        // Get timestamp.
        double stamp = global_odom_msg->header.stamp.toSec();
        // Get pose.
        const Eigen::Affine3d pose =
            dockslam::EigenAffineFromOdom(*global_odom_msg);
        // Get vel.
        const Eigen::Vector3d vel =
            dockslam::EigenVelFromOdom(*global_odom_msg);
        // Get covariance.
        const Eigen::Matrix<double, 6, 6> cov =
            dockslam::Eigen6x6PoseCovFromPoseCov(global_odom_msg->pose);
        // Send to graph.
        gm_->AddTargetGlobalState(
            pose, vel, dockslam::GtsamPoseCovarianceFromRos(cov), stamp);

        if (is_first_target_odom) {
          gm_->SetupPriors();
          is_first_target_odom = false;
        }
      }
    // ------- Target global navigation ---------

    // ------- Chaser global navigation ---------
    } else if ((m.getTopic() == chaser_global_odom_topic) ||
               ("/" + m.getTopic() == chaser_global_odom_topic)) {
      nav_msgs::Odometry::ConstPtr global_odom_msg =
          m.instantiate<nav_msgs::Odometry>();
      if (global_odom_msg != nullptr) {
        ch_global_odom_count++;

        // Get timestamp.
        double stamp = global_odom_msg->header.stamp.toSec();
        // Get pose.
        Eigen::Affine3d pose = dockslam::EigenAffineFromOdom(*global_odom_msg);
        // Get covariance.
        const Eigen::Matrix<double, 6, 6> cov =
            dockslam::Eigen6x6PoseCovFromPoseCov(global_odom_msg->pose);
        // Send to graph.
        gm_->AddChaserGlobalPose(
            pose, dockslam::GtsamPoseCovarianceFromRos(cov), stamp);
      }
    // ------- Chaser global navigation ---------

    // ------- Chaser optical pose ---------
    } else if ((m.getTopic() == chaser_optical_topic) ||
               ("/" + m.getTopic() == chaser_optical_topic)) {
      geometry_msgs::PoseWithCovarianceStamped::ConstPtr optical_msg =
          m.instantiate<geometry_msgs::PoseWithCovarianceStamped>();
      if (optical_msg != nullptr) {
        ch_optical_count++;

        // Get timestamp.
        double stamp = optical_msg->header.stamp.toSec();
        // Get pose.
        const Eigen::Affine3d pose =
            dockslam::EigenAffineFromPoseCovStamped(*optical_msg);
        // Get noise.
        const Eigen::Matrix<double, 6, 6> cov =
            dockslam::Eigen6x6PoseCovFromPoseCov(optical_msg->pose);
        // Send to graph.
        gm_->AddChaserRelativeOpticalPose(
            pose, dockslam::GtsamPoseCovarianceFromRos(cov), stamp);

      }
    // ------- Chaser optical pose ---------

    // ------- Target usbl position ---------
    } else if ((m.getTopic() == target_usbl_topic) ||
               ("/" + m.getTopic() == target_usbl_topic)) {
      visualization_msgs::Marker::ConstPtr usbl_msg =
          m.instantiate<visualization_msgs::Marker>();
      if (usbl_msg != nullptr) {
        tgt_usbl_count++;

        // Get timestamp.
        double stamp = usbl_msg->header.stamp.toSec();
        // Get position.
        const Eigen::Vector3d position = dockslam::EigenPositionFromMarker(*usbl_msg);
        // Send to graph.
        gm_->AddTargetUsblFix(position, stamp);

        i++;
      }
    // ------- Target usbl position ---------
    //
    // ------- Target imu data ---------
    } else if ((m.getTopic() == target_imu_topic) ||
               ("/" + m.getTopic() == target_imu_topic)) {
        sbg_driver::SbgImuData::ConstPtr imu_msg =
          m.instantiate<sbg_driver::SbgImuData>();
      if (imu_msg != nullptr) {
        tgt_imu_count++;

        double stamp = imu_msg->header.stamp.toSec();
        const Eigen::Vector3d imu_acc{imu_msg->accel.x, imu_msg->accel.y,
                                      imu_msg->accel.z};
        const Eigen::Vector3d imu_gyro{imu_msg->gyro.x, imu_msg->gyro.y,
                                       imu_msg->gyro.z};
        // Send to graph.
        gm_->AddTargetImu(imu_acc, imu_gyro, stamp);
      }
    // ------- Target imu data ---------

    // ------- Target gps position ---------
    } else if ((m.getTopic() == target_gps_topic) ||
               ("/" + m.getTopic() == target_gps_topic)) {
        sbg_driver::SbgGpsPos::ConstPtr gps_msg =
          m.instantiate<sbg_driver::SbgGpsPos>();
      if (gps_msg != nullptr) {
        tgt_gps_count++;

        double stamp = gps_msg->header.stamp.toSec();

        // Convert lat-lon to UTM and offset.
        int zone;
        bool northp;
        double x, y, z;
        GeographicLib::UTMUPS::Forward(gps_msg->latitude, gps_msg->longitude,
                                       zone, northp, x, y);
        const Eigen::Vector3d gps_pos{x - UTM_X, y - UTM_Y, gps_msg->altitude};
        const Eigen::Vector3d gps_stddev{gps_msg->position_accuracy.x,
                                         gps_msg->position_accuracy.y,
                                         gps_msg->position_accuracy.z};
        // Send to graph.
        gm_->AddTargetGps(gps_pos, gps_stddev, stamp);

      }
    // ------- Target gps position ---------

    // ------- Target hdt position ---------
    } else if ((m.getTopic() == target_hdt_topic) ||
               ("/" + m.getTopic() == target_hdt_topic)) {
        sbg_driver::SbgGpsHdt::ConstPtr hdt_msg =
          m.instantiate<sbg_driver::SbgGpsHdt>();
      if (hdt_msg != nullptr) {
        tgt_hdt_count++;

        double stamp = hdt_msg->header.stamp.toSec();

        // Angles are in NED with values from 0 to 360 degs.
        double heading = hdt_msg->true_heading;
        // NED -> ENU rotate by +90 degrees to that 0 points East, and add
        // negative sign since Z is pointing up in ENU.
        heading = 90.0 - heading;
        // Make sure it's within 0-360, and map it to -180 to 180.
        heading = std::fmod(heading, 360.0);
        if (heading > 180.0) {
          heading -= 360.0;
        }

        double heading_stddev = (hdt_msg->true_heading_acc) * M_PI / 180.0;
        // Pitch to ENU as a well, add minus sign.
        double pitch = -hdt_msg->pitch;
        double pitch_stddev = hdt_msg->pitch_acc * M_PI / 180.0;
        // Send to graph.
        gm_->AddTargetHdt(heading * M_PI / 180.0, heading_stddev,
                          pitch * M_PI / 180.0, pitch_stddev, stamp);
      }
    }
    // ------- Target hdt position ---------

  }

  std::cout << "Got this many chaser global odom messages: "
            << ch_global_odom_count << std::endl;
  std::cout << "Got this many chaser optical messages: " << ch_optical_count
            << std::endl;
  std::cout << "Got this many target usbl messages: " << tgt_usbl_count
            << std::endl;
  std::cout << "Got this many target imu messages: " << tgt_imu_count
            << std::endl;
  std::cout << "Got this many target gps messages: " << tgt_gps_count
            << std::endl;
  std::cout << "Got this many target hdt messages: " << tgt_hdt_count
            << std::endl;


  gm_->Print();
  gm_->PrintInitialEstimates();

  // Optimize the graph.
  const gtsam::Values results = gm_->OptimizeOnce();
  results.print("----------- RESULTS ------------");

  // Save graph .dot file.
  std::ofstream filename("graph.dot");
  gm_->SaveGraph(filename);

  return 0;
} // End main.
