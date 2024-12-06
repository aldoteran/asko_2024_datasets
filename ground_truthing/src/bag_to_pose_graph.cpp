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
#include "utils/tfm_utils.h"
#include "utils/ros_gtsam_utils.h"
#include "backend/graph_manager.h"

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

Eigen::Matrix2d ToRotMatrix(const double angle_rad) {
  const double cs = std::cos(angle_rad);
  const double sn = std::sin(angle_rad);
  Eigen::Matrix2d rot_mat;
  rot_mat << cs, -sn, sn, cs;
  return rot_mat;
}

void SetupConfig(dockslam::GraphManager::Config &config, ros::NodeHandle &nh) {
  // Priors.
  nh.getParam("prior_roll_stddev", config.target_init_pose_stddev(0));
  nh.getParam("prior_pitch_stddev", config.target_init_pose_stddev(1));
  nh.getParam("prior_yaw_stddev", config.target_init_pose_stddev(2));
  nh.getParam("prior_x_stddev", config.target_init_pose_stddev(3));
  nh.getParam("prior_y_stddev", config.target_init_pose_stddev(4));
  nh.getParam("prior_z_stddev", config.target_init_pose_stddev(5));
  nh.getParam("prior_vel_x_stddev", config.target_init_vel_stddev(0));
  nh.getParam("prior_vel_y_stddev", config.target_init_vel_stddev(1));
  nh.getParam("prior_vel_z_stddev", config.target_init_vel_stddev(2));

  // Previously estimated IMU biases.
  nh.getParam("init_accel_bias_x", config.init_accel_bias(0));
  nh.getParam("init_accel_bias_y", config.init_accel_bias(1));
  nh.getParam("init_accel_bias_z", config.init_accel_bias(2));
  nh.getParam("init_accel_bias_stddev", config.init_accel_bias_stddev);
  nh.getParam("init_omega_bias_x", config.init_omega_bias(0));
  nh.getParam("init_omega_bias_y", config.init_omega_bias(1));
  nh.getParam("init_omega_bias_z", config.init_omega_bias(2));
  nh.getParam("init_omega_bias_stddev", config.init_omega_bias_stddev);

  // Measurement noises. USBL:
  nh.getParam("usbl_x_stddev", config.usbl_noise_stddev(0));
  nh.getParam("usbl_y_stddev", config.usbl_noise_stddev(1));
  nh.getParam("usbl_z_stddev", config.usbl_noise_stddev(2));
  // Optical pose.
  nh.getParam("optical_rot_stddev", config.optical_rot_stddev);
  nh.getParam("optical_trans_stddev", config.optical_trans_stddev);

  // IMU noise model parameters.
  nh.getParam("accel_noise_x", config.imu_accel_noise_stddev(0));
  nh.getParam("accel_noise_y", config.imu_accel_noise_stddev(1));
  nh.getParam("accel_noise_z", config.imu_accel_noise_stddev(2));
  nh.getParam("accel_bias_x", config.imu_accel_bias_stddev(0));
  nh.getParam("accel_bias_y", config.imu_accel_bias_stddev(1));
  nh.getParam("accel_bias_z", config.imu_accel_bias_stddev(2));
  nh.getParam("omega_noise_x", config.imu_omega_noise_stddev(0));
  nh.getParam("omega_noise_y", config.imu_omega_noise_stddev(1));
  nh.getParam("omega_noise_z", config.imu_omega_noise_stddev(2));
  nh.getParam("omega_bias_x", config.imu_omega_bias_stddev(0));
  nh.getParam("omega_bias_y", config.imu_omega_bias_stddev(1));
  nh.getParam("omega_bias_z", config.imu_omega_bias_stddev(2));
  nh.getParam("imu_frequency", config.imu_frequency);

  // Extrinsic calibrations.
  //nh.getParam("camera_extrinsics_roll", config.chaser_camera_extrinsics(0));
  //nh.getParam("camera_extrinsics_pitch", config.chaser_camera_extrinsics(1));
  //nh.getParam("camera_extrinsics_yaw", config.chaser_camera_extrinsics(2));
  nh.getParam("camera_extrinsics_qx",
              config.chaser_camera_extrinsics(0));
  nh.getParam("camera_extrinsics_qy",
              config.chaser_camera_extrinsics(1));
  nh.getParam("camera_extrinsics_qz",
              config.chaser_camera_extrinsics(2));
  nh.getParam("camera_extrinsics_qw",
              config.chaser_camera_extrinsics(3));
  nh.getParam("camera_extrinsics_x", config.chaser_camera_extrinsics(4));
  nh.getParam("camera_extrinsics_y", config.chaser_camera_extrinsics(5));
  nh.getParam("camera_extrinsics_z", config.chaser_camera_extrinsics(6));
  nh.getParam("ch_usbl_extrinsics_roll", config.chaser_usbl_extrinsics(0));
  nh.getParam("ch_usbl_extrinsics_pitch", config.chaser_usbl_extrinsics(1));
  nh.getParam("ch_usbl_extrinsics_yaw", config.chaser_usbl_extrinsics(2));
  nh.getParam("ch_usbl_extrinsics_x", config.chaser_usbl_extrinsics(3));
  nh.getParam("ch_usbl_extrinsics_y", config.chaser_usbl_extrinsics(4));
  nh.getParam("ch_usbl_extrinsics_z", config.chaser_usbl_extrinsics(5));
  //nh.getParam("fiducials_extrinsics_roll",
              //config.target_fiducials_extrinsics(0));
  //nh.getParam("fiducials_extrinsics_pitch",
              //config.target_fiducials_extrinsics(1));
  //nh.getParam("fiducials_extrinsics_yaw",
              //config.target_fiducials_extrinsics(2));
  nh.getParam("fiducials_extrinsics_qx",
              config.target_fiducials_extrinsics(0));
  nh.getParam("fiducials_extrinsics_qy",
              config.target_fiducials_extrinsics(1));
  nh.getParam("fiducials_extrinsics_qz",
              config.target_fiducials_extrinsics(2));
  nh.getParam("fiducials_extrinsics_qw",
              config.target_fiducials_extrinsics(3));
  nh.getParam("fiducials_extrinsics_x", config.target_fiducials_extrinsics(4));
  nh.getParam("fiducials_extrinsics_y", config.target_fiducials_extrinsics(5));
  nh.getParam("fiducials_extrinsics_z", config.target_fiducials_extrinsics(6));
  nh.getParam("tgt_usbl_extrinsics_roll", config.target_usbl_extrinsics(0));
  nh.getParam("tgt_usbl_extrinsics_pitch", config.target_usbl_extrinsics(1));
  nh.getParam("tgt_usbl_extrinsics_yaw", config.target_usbl_extrinsics(2));
  nh.getParam("tgt_usbl_extrinsics_x", config.target_usbl_extrinsics(3));
  nh.getParam("tgt_usbl_extrinsics_y", config.target_usbl_extrinsics(4));
  nh.getParam("tgt_usbl_extrinsics_z", config.target_usbl_extrinsics(5));
}

void PrintParameterServer(ros::NodeHandle &nh){
    std::vector<std::string> keys;
    nh.getParamNames(keys);

    std::cout << "------ PARAMETER SERVER ------\n";
    for (std::string key : keys){
        double val;
        nh.getParam(key, val);
        std::cout << key << ": " << val << "\n";
    }
}

int main(int argc, char *argv[]){

  ros::init(argc, argv, "bag_to_graph");
  ros::NodeHandle nh_;
  PrintParameterServer(nh_);

  bool debug;
  nh_.getParam("debug", debug);

  // Initialize graph here with a config file from the input aguments.
  dockslam::GraphManager::Config graph_config;
  SetupConfig(graph_config, nh_);
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
  const std::string target_usbl_topic = "/service_boat/usbl_fix";
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

  std::cout << "Processing " << view.size() << " messages." << std::endl;
  int ch_global_odom_count = 0;
  int ch_optical_count = 0;
  int tgt_global_odom_count = 0;
  int tgt_usbl_count = 0;
  int tgt_imu_count = 0;
  int tgt_gps_count = 0;
  int tgt_hdt_count = 0;

  // FIXME: Optical outliers for dataset 17.
  std::vector<int> optical_outliers;
  nh_.getParam("optical_outliers", optical_outliers);
  std::vector<double> optical_outlier_times = {
      1718120541.800157, 1718120542.006601, 1718120542.801705,
      1718120543.447691, 1718120543.938742, 1718120544.551751,
      1718120500.551715, 1718120500.866524, 1718120501.318398,
      1718120501.541663, 1718120501.725677, 1718120501.897093,
      1718120502.14488,  1718120502.379539, 1718120500.304396,
      1718120502.612497, 1718120122.485302, 1718120123.291622,
      1718120122.260119, 1718120083.268651, 1718120342.179019,
      1718120341.7517};
  std::vector<double> usbl_outlier_times = {
      1718120264.332745, 1718120035.804216, 1718120230.522205,
      1718120231.20116,  1718120233.444929, 1718120219.133944,
      1718120543.998107, 1718120544.627634, 1718120117.463796,
      1718120116.818518, 1718120081.143286, 1718120081.838988,
      1718120083.421746, 1718120078.649655};

  // Manual altitude measurement for the target. This value was
  // measured during the campaign and will be used as evidence
  // for the optimization.
  double target_altitude;
  double target_altitude_stddev;
  nh_.getParam("manual_altitude", target_altitude);
  nh_.getParam("manual_altitude_stddev", target_altitude_stddev);

  // Constant UTM offset. What's the best way to do this?
  double UTM_X = 651000.5;
  double UTM_Y = 6523400.5;

  bool is_first_target_odom = true;
  size_t i = 0;
  // Main for loop. Here we'll get all the data and build the graph.
  BOOST_FOREACH (rosbag::MessageInstance const m, view) {
    //if (i > 1000) {
      //break;
    //}

    // ------- target global navigation ---------
    if ((m.getTopic() == target_global_odom_topic) ||
        ("/" + m.getTopic() == target_global_odom_topic)) {
      nav_msgs::Odometry::ConstPtr global_odom_msg =
          m.instantiate<nav_msgs::Odometry>();
      if (global_odom_msg != nullptr) {
        tgt_global_odom_count++;

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

        if (!gm_->IsGraphInit()) {
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

        // Reject previously detected outliers.
        auto found = std::find(optical_outliers.begin(), optical_outliers.end(),
                               ch_optical_count - 1);
        if (found != optical_outliers.end()) {
          std::cout << "Filtering optical outlier at idx " << ch_optical_count - 1
                    << "\n";
          continue;
        }

        // Get timestamp.
        double stamp = optical_msg->header.stamp.toSec();

        // Get pose.
        const Eigen::Affine3d pose =
            dockslam::EigenAffineFromPoseCovStamped(*optical_msg);

        if (debug) {
          auto t = pose.translation();
          auto gp = gtsam::Pose3(pose.matrix());
          auto q = gp.rotation().toQuaternion();
          std::cout.precision(16);
          std::cout << ch_optical_count << "," << stamp << "," << q.x() << "," << q.y() << "," << q.z() << ","
                    << q.w() << "," << t(0) << "," << t(1) << "," << t(2)
                    << "\n";
          continue;

        }

        // Get noise.
        const Eigen::Matrix<double, 6, 6> cov =
            dockslam::Eigen6x6PoseCovFromPoseCov(optical_msg->pose);
        // Send to graph.
        //gm_->AddChaserRelativeOpticalPose(
            //pose, dockslam::GtsamPoseCovarianceFromRos(cov), stamp);
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

        auto found =
            std::find(usbl_outlier_times.begin(),
                      usbl_outlier_times.end(), tgt_usbl_count - 1);
        if (found != usbl_outlier_times.end()) {
          std::cout << "Filtering usbl outlier (with stamp) at idx " << tgt_usbl_count - 1
                    << "\n";
          continue;
        }

        // Get position.
        Eigen::Vector3d position = dockslam::EigenPositionFromMarker(*usbl_msg);
        // FIXME: Let's try to keep the measurements underwater.
        //position(2) = std::abs(position(2)) * (-1);
        // Send to graph.
        gm_->AddTargetUsblFix(position, stamp);
      }
    // ------- Target usbl position ---------

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
        // Since we measured the height of the AHRS wrt the water surface,
        // we'll add it as evidence with an appropriate noise in the GPSFactor.
        const Eigen::Vector3d gps_pos{x - UTM_X, y - UTM_Y, target_altitude};
        const Eigen::Vector3d gps_stddev{gps_msg->position_accuracy.x,
                                         gps_msg->position_accuracy.y,
                                         target_altitude_stddev};
        // Send to graph.
        gm_->AddTargetGps(gps_pos, gps_stddev, stamp);

        i++;
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
        double heading_deg =
            dockslam::NedHeadingInEnu((hdt_msg->true_heading), true);
        double heading_stddev = (hdt_msg->true_heading_acc) * M_PI / 180.0;
        // Pitch to ENU as a well, add minus sign.
        double pitch_deg = -hdt_msg->pitch;
        double pitch_stddev = hdt_msg->pitch_acc * M_PI / 180.0;

        // Send to graph.
        gm_->AddTargetHdt(heading_deg * M_PI / 180.0, heading_stddev,
                          pitch_deg * M_PI / 180.0, pitch_stddev, stamp);
      }
    }
    // ------- Target hdt position ---------

  }

  if (debug) {
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
  }

  gm_->Print();
  const std::string path_to_data =
      //"/home/aldoteran/docking_ws/src/asko_2024_datasets/ground_truthing/";
      "/tmp/";
  const gtsam::Values results = gm_->PrintISAM2Results(path_to_data);

  // Append the results to the rosbag if necessary.
  bool append_to_bag;
  nh_.getParam("append_results", append_to_bag);
  if (append_to_bag){
      const std::map<int, double> stamps = gm_->GetTimestamps();
      const std::string ground_truth_topic = "/service_boat/gt";
      const std::string global_frame = "/map";
      const std::string body_frame = "/service_boat/gt/sbg_link";
      ros_gtsam::AppendGTSAMResultsToRosbag(bagfile, ground_truth_topic,
                                            global_frame, body_frame, results,
                                            stamps);
  }

  return 0;
} // End main.
