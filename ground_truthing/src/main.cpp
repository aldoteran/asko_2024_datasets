/* ----------------------------------------------------------------------------

 * Underwater Fun Research Co.,
 * Seattle - Stockholm - Örebro
 * Authors: Aldo Teran Espinoza, Antonio Teran Espinoza, David Baxter.

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file main.cpp
 * @brief Read rosbag, build a graph, optimize, save graph and results.
 * @date September 18, 2024
 * @author Aldo Teran Espinoza
 * @author_email aldot@kth.se
 */
#include <iostream>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "utils/ros_eigen_conversions.h"

//#include <graph_manager.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

int main(int argc, char *argv[]){

    std::cout << "skit" << std::endl;

  if (argc != 3) {
    throw std::runtime_error("Only the path to the rosbag and the config file "
                             "can be used as argument. Try again!");
  }

  // Initialize graph here with a config file from the input aguments.
  //GraphManager::Config graph_config;
  /*
   * TODO: Setup Config file parameters
   */
  //std::unique_ptr<GraphManager> gm_ = std::make_unique<GraphManager>(graph_config);

  // Read bag.
  rosbag::Bag bag;
  std::cout << "Reading bag: " << argv[1] << std::endl;
  bag.open(argv[1], rosbag::bagmode::Read);

  // Add topics we want to read into a topics list and query the topics.
  const std::string chaser_global_odom_topic = "/lolo/dr/odom";
  const std::string chaser_odom_topic = "/lolo/dr/relative/odom";
  const std::string chaser_optical_topic = "/lolo/perception/optical_pose";
  const std::string target_usbl_topic = "/service_boat/usbl_fix";
  const std::string target_odom_topic = "/service_boat/dr/odom";
  // TODO: do we need the relative odom for the target ad well?
  std::vector<std::string> topics;
  topics.push_back(chaser_global_odom_topic);
  topics.push_back(chaser_odom_topic);
  topics.push_back(chaser_optical_topic);
  topics.push_back(target_usbl_topic);
  topics.push_back(target_odom_topic);
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  // (debug)
  std::cout << "Processing " << view.size() << " messages." << std::endl;
  int ch_global_odom_count = 0;
  int ch_odom_count = 0;
  int ch_optical_count = 0;
  int tgt_odom_count = 0;
  int tgt_usbl_count = 0;

  // Main for loop. Here we'll get all the data and build the graph.
  BOOST_FOREACH (rosbag::MessageInstance const m, view) {

    // LoLo relative odometry msg.
    if ((m.getTopic() == chaser_odom_topic) ||
        ("/" + m.getTopic() == chaser_odom_topic)) {
      nav_msgs::Odometry::ConstPtr odom_msg =
          m.instantiate<nav_msgs::Odometry>();
      if (odom_msg != nullptr) {
        ch_odom_count++;

        // Get pose.
        const Eigen::Affine3d pose =
            EigenAffineFromOdom(*odom_msg);
        // TODO: get velocities?
        // Get noise.
        const gtsam::noiseModel::Gaussian::shared_ptr noise =
            gtsam::noiseModel::Gaussian::Covariance(
                Eigen6x6PoseCovFromPoseCov(odom_msg->pose));
        // TODO: Send to graph.
        //gm_->AddChaserRelativeOdom(pose, noise, /*velocities*/);
      }
    } else if ((m.getTopic() == chaser_global_odom_topic) ||
               ("/" + m.getTopic() == chaser_global_odom_topic)) {
      nav_msgs::Odometry::ConstPtr global_odom_msg =
          m.instantiate<nav_msgs::Odometry>();
      if (global_odom_msg != nullptr) {
        ch_global_odom_count++;

        // Get pose.
        const Eigen::Affine3d pose =
            EigenAffineFromOdom(*global_odom_msg);
        // TODO: get velocities?
        // Get noise.
        const gtsam::noiseModel::Gaussian::shared_ptr noise =
            gtsam::noiseModel::Gaussian::Covariance(
                Eigen6x6PoseCovFromPoseCov(global_odom_msg->pose));
        // TODO: Send to graph.
        //gm_->AddChaserGlobalOdom(3dPoseMeasFromOdometry(chaser_odom));
      }
    } else if ((m.getTopic() == chaser_optical_topic) ||
               ("/" + m.getTopic() == chaser_optical_topic)) {
      geometry_msgs::PoseWithCovarianceStamped::ConstPtr optical_msg =
          m.instantiate<geometry_msgs::PoseWithCovarianceStamped>();
      if (optical_msg != nullptr) {
        ch_optical_count++;

        // Get pose.
        const Eigen::Affine3d pose = EigenAffineFromPoseCovStamped(*optical_msg);
        // Get noise.
        gtsam::noiseModel::Gaussian::shared_ptr noise =
            gtsam::noiseModel::Gaussian::Covariance(
                Eigen6x6PoseCovFromPoseCov(optical_msg->pose));
        // TODO: Send to graph.
        //gm_->AddChaserRelativeOpticalPose(3dPoseMeasFromPoseCovStamped(optical_msg));
      }
    } else if ((m.getTopic() == target_odom_topic) ||
               ("/" + m.getTopic() == target_odom_topic)) {
      nav_msgs::Odometry::ConstPtr target_odom_msg =
          m.instantiate<nav_msgs::Odometry>();
      if (target_odom_msg != nullptr) {
        tgt_odom_count++;

        // Get pose.
        const Eigen::Affine3d pose = EigenAffineFromOdom(*target_odom_msg);
        // Get noise.
        gtsam::noiseModel::Gaussian::shared_ptr noise =
            gtsam::noiseModel::Gaussian::Covariance(
                Eigen6x6PoseCovFromPoseCov(target_odom_msg->pose));
        // TODO: Send to graph.
        //gm_->AddTargetGlobalOdom(3dPoseMeasFromOdometry(target_odom));
      }
    } else if ((m.getTopic() == target_usbl_topic) ||
               ("/" + m.getTopic() == target_usbl_topic)) {
      visualization_msgs::Marker::ConstPtr usbl_msg =
          m.instantiate<visualization_msgs::Marker>();
      if (usbl_msg != nullptr) {
        tgt_usbl_count++;

        // Get position.
        const gtsam::Point3 position = EigenPositionFromMarker(*usbl_msg);
        // Turn it into range.
        const double range = position.norm();
        // Get unit vector of bearing.
        const gtsam::Unit3 bearing = gtsam::Unit3::FromPoint3(position);
        // TODO: Send to graph.
        //gm_->AddTargetRangeBearing(3dRangeBearingFromMarker(target_usbl_meas));
      }
    }

  }

  /*
   * TODO:
   * Save graph.
   * Optimize.
   * Optimize.
   * Optimize.
   * Save results.
   * Print something smart about it.
   */

  std::cout << "Got this many chaser global odom messages: "
            << ch_global_odom_count << std::endl;
  std::cout << "Got this many chaser odom messages: " << ch_odom_count
            << std::endl;
  std::cout << "Got this many chaser optical messages: " << ch_optical_count
            << std::endl;
  std::cout << "Got this many target odom messages: " << tgt_odom_count
            << std::endl;
  std::cout << "Got this many target usbl messages: " << tgt_usbl_count
            << std::endl;

  return 0;
} // End main.
