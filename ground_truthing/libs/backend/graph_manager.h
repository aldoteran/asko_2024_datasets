/**
 * @file graph_manager.h
 * @brief Factor graph manager for dataset optimization.
 * @date Oct 1, 2024
 * @author aldo terán (aldot@kth.se)
 * @author tonio terán (teran@mit.edu)
 */

#ifndef _BACKEND_GRAPH_MANAGER_H_
#define _BACKEND_GRAPH_MANAGER_H_

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam_unstable/slam/PartialPriorFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include "utils/csv_utils.h"

#include <vector>
#include <iostream>
#include <fstream>

using gtsam::symbol_shorthand::X; // Target's pose to optimize.
using gtsam::symbol_shorthand::V; // Target's velocity to optimize.
using gtsam::symbol_shorthand::B; // Target's imu bias to optimize.

namespace dockslam {
/*!
  Backend inference class for SLAM using GTSAM. Optimization parameters
  and noise characterisicts are set within `config/params.yaml`.
 */
class GraphManager {
public:
  struct Config {
    gtsam::Vector6 target_init_pose_stddev; // [r,p,y,x,y,z] [rad,m]
    gtsam::Vector3 target_init_vel_stddev;  // [x,y,z] [m]
    gtsam::Vector6 init_imu_bias_stddev;
    gtsam::Vector3 imu_accel_noise_stddev;
    gtsam::Vector3 imu_omega_noise_stddev;
    gtsam::Vector3 imu_accel_bias_stddev;
    gtsam::Vector3 imu_omega_bias_stddev;
    gtsam::Vector3 init_accel_bias;
    gtsam::Vector3 init_omega_bias;
    double imu_frequency;             // [hz]
    gtsam::Vector3 usbl_noise_stddev; // [x,y,z] in [m]
    gtsam::Vector6 chaser_camera_extrinsics; // [r,p,y,x,y,z]
    gtsam::Vector6 chaser_usbl_extrinsics;
    gtsam::Vector6 target_fiducials_extrinsics;
    gtsam::Vector6 target_usbl_extrinsics;
  };

  // Used as input to the TargetUsblFactor.
  struct UsblMeasurement {
    gtsam::Point3 t_tu_trans_cu; // Raw measurement. T_Tusbl_trans_Cusbl.
    gtsam::Pose3 w_tfm_c;        // W_tfm_C.
    gtsam::Pose3 c_tfm_cu;       // C_tfm_Cusbl.
    gtsam::Pose3 t_tfm_tu;       // T_tfm_Tusbl.
  };

  // Instantiate the graph manager using a config struct.
  explicit GraphManager(const Config &config);

  // Destructor.
  ~GraphManager();

  // Setup the Target's IMU odometer.
  void SetupImuOdometer();
  // Setup the prior factors.
  void SetupPriors();
  // Setup the sensor extrinsic calibrations.
  void SetupExtrinsics();
  // Setup the known noise models..
  void SetupNoises();

  void AddTargetGlobalState(const Eigen::Affine3d &pose,
                           const Eigen::Vector3d &vel,
                           const Eigen::Matrix<double, 6, 6> &cov,
                           double stamp);

  // Store the pose and covartiance of the chaser in the global (ENU)
  // frame of reference.
  void AddChaserGlobalPose(const Eigen::Affine3d &pose,
                           const Eigen::Matrix<double, 6, 6> &cov,
                           double stamp);

  // Composes the optical pose with the chaser's current pose and
  // adds a Pose3 unary factor to the target's state.
  void AddChaserRelativeOpticalPose(const Eigen::Affine3d &pose,
                                    const Eigen::Matrix<double, 6, 6> &cov,
                                    double stamp);

  // Rotate the usbl position to express it wrt the chaser's frame
  // and add it as a Point3 unary factor to the target's state.
  void AddTargetUsblFix(const Eigen::Vector3d &pos, double stamp);

  // Adds target's imu accelerations and angular velocities to the
  // preintegrated imu odometer.
  void AddTargetImu(const Eigen::Vector3d &acc, const Eigen::Vector3d &gyro,
                    double stamp);

  // Adds a GPS unary factor to the target's state.
  void AddTargetGps(const Eigen::Vector3d &gps_pos,
                    const Eigen::Vector3d &stddev, double stamp);

  // Adds the GPS' true heading and pitch as unary factor to
  // the target's state.
  void AddTargetHdt(double heading, double heading_stddev, double pitch,
                    double pitch_stddev, double stamp);

  // Uses the preintegrated IMU measurements to add a BetweenFactor
  // between the previous and the current frame and adds initial conditions
  // for optimization.
  void AddImuBetweenFactor();

  void SaveGraph(std::ofstream &filename);

  // Optimize the graph once and return the values.
  gtsam::Values OptimizeOnce();

  // Print the graph.
  void Print();

  // Print initial estimates.
  void PrintInitialEstimates();

private:
  void InitializeGraph();

  Config config_;

  // Factor graph to arrange all observations.
  gtsam::NonlinearFactorGraph graph_;

  // Values to capture initial estimates for the new nodes.
  gtsam::Values initial_estimates_;

  // Contains the latest of the chaser global poses.
  gtsam::Pose3 chaser_global_pose_;
  gtsam::noiseModel::Gaussian::shared_ptr chaser_global_noise_;
  double chaser_pose_stamp_ = 0;

  // Contains the most recently measured target's odometry.
  gtsam::Pose3 target_odom_pose_;
  // Same for the velocity.
  gtsam::Vector3 target_odom_vel_;
  // Contains the most recently predicted target's global pose.
  gtsam::Pose3 target_global_pose_;
  // Same for the velocity.
  gtsam::Vector3 target_vel_;
  // Target's global uncertainty.
  gtsam::noiseModel::Gaussian::shared_ptr target_pose_noise_;
  // Target's vel velocity uncertainty.
  gtsam::noiseModel::Diagonal::shared_ptr target_vel_noise_;
  // Target's usbl manually tuned uncertainty.
  gtsam::noiseModel::Diagonal::shared_ptr target_usbl_noise_;
  // Target's preint imu odometer.
  std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> odometer_ = nullptr;
  // Current IMU bias.
  gtsam::imuBias::ConstantBias imu_bias_;
  // IMU bias noise.
  gtsam::noiseModel::Diagonal::shared_ptr imu_bias_noise_;
  // Last measurement's stamp.
  double prev_imu_stamp_ = 0;

  // Sensor extrinsic calibrations.
  gtsam::Pose3 chaser_camera_extr_; // C_tfm_cam.
  gtsam::Pose3 chaser_usbl_extr_; // C_tfm_cusbl.
  gtsam::Pose3 target_usbl_extr_; // T_tfm_tusbl.
  gtsam::Pose3 target_fiducial_extr_; // T_tfm_fiducial.

  // Variable for keeping track of the number of keyframes in the graph.
  // Counter should be increased at the end of the keyframed function.
  int cur_frame_ = 0;

  bool is_chaser_init = false;
  bool is_target_init = false;
  bool is_graph_init = false;

  // Raw measruements by keyframe fro csv file.
  std::vector<std::string> csvdata_;
};

} // namespace dockslam.

#endif //_BACKEND_GRAPH_MANAGER_H_
