/**
 * @file GraphManager.h
 * @brief Factor graph manager for dataset optimization.
 * @date Oct 1, 2024
 * @author aldo terán (aldot@kth.se)
 * @author tonio terán (teran@mit.edu)
 */

#ifndef GRAPH_MANAGER_H_
#define GRAPH_MANAGER_H_

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/nonlinear/Marginals.h>

#include <vector>
#include <iostream>

using gtsam::symbol_shorthand::X; // chaser's state.
using gtsam::symbol_shorthand::S; // target's position as 3D landmark.
using gtsam::symbol_shorthand::T; // target's full 6DoF pose.
using gtsam::symbol_shorthand::L; // target's USBL->lights extrinsic calib.
using gtsam::symbol_shorthand::U; // target's vel.
using gtsam::symbol_shorthand::H; // target's heading.

namespace dockslam {

/*!
  Backend inference class for SLAM using GTSAM/iSAM2. Optimization parameters
  and noise characterisicts are set within `params/slam_params.yaml`.
 */
class GraphManager {
 public:
  struct Config {
    // Prior noise on the chaser's position [m].
    double prior_x_stddev = 2.0;
    double prior_y_stddev = 2.0;
    double prior_z_stddev = 0.01;
    // Prior noise on the chaser's orientation [rad].
    double prior_roll_stddev = 0.01;
    double prior_pitch_stddev = 0.01;
    double prior_yaw_stddev = 0.5;
    // Prior noise on the chaser's linear velocity [m/s].
    double prior_vel_x_stddev = 0.01;
    double prior_vel_y_stddev = 0.1;
    double prior_vel_z_stddev = 0.1;

    // Chaser's camera extrinsic calibration.
    Eigen::Matrix<double, 6, 1> camera_extrinsics;

    // Prior noise on the target's position [m].
    double prior_target_pos_stddev = 5.0;
    // Prior noise on the target's velocity [m/s].
    double prior_target_vel_stddev = 1.0;
    // Prior noise on the target's heading angle [rad].
    double prior_target_head_stddev = 0.01;

    // USBL noise characterisicts.
    // Fraction of the total measured range determining 1 sigma for all three
    // components (bearing, elevation, range). See slam_config.yaml file.
    Eigen::Vector3d usbl_error;

    // Motion model position noise [m].
    double motion_model_pos_stddev = 1.0;
    // Motion model velocity noise [m/s].
    double motion_model_vel_stddev = 0.5;
    // Motion model heading noise [rad].
    double motion_model_head_stddev = 0.01;

    // FIXME: this shouldn't actually by chosen by us I think,
    // we should use the marginal covariance of the postion and
    // the angle to calculate it, but sounds like a lot of work atm.
    // Increased motion model rotation stddev [rad].
    double increased_motion_rot_stddev = 0.00001;
    // Increased motion model position stddev [m].
    double increased_motion_pos_stddev = 0.0001;
    // FIXME: same goes for this ones, what's the best way of assigning
    // a noise to this factor?
    // Rotation stddev for target's pose between factor [rad].
    double target_pose_between_rot_stddev = 0.00001;
    // Position stddev for target's pose between factor [m].
    double target_pose_between_pos_stddev = 0.0001;

    // Extrinsic calibration from the USBL frame, which in our case is same
    // as the base_link, and the frame defined at the center of the light
    // beacons. T_tfm_L = [roll, pitch, yaw, x, y, z].
    Eigen::Matrix<double, 6, 1> target_extrinsics;
    // We'll assign a super tiny noise on it to make sure that it stays rigid.
    double target_extrinsics_noise = 1e-6;
  };

  explicit GraphManager(const Config& config);

  // Return Graph status.
  inline bool IsGraphInit() const { return fg_init_; }

  // Return flag for target prior initialization.
  inline bool IsTargetInit() const { return is_target_init_; }

  // Return flag for target prior initialization.
  inline bool IsChaserInit() const { return is_chaser_init_; }

  // Return flag for target's visibility.
  inline bool IsTargetVisible() const { return is_target_visible_; }

  // Return the most recent optimized pose of the chaser composed with
  // with the accumulated odometry measurement at the time. This
  // pose is the same as the one introduced as intial value before
  // the optimizing the graph.
  inline const Eigen::Affine3d GetChaserPose() const {
      return Eigen::Affine3d(cur_pose_estimate_.matrix());
  }

  // Return the most recently optimized value for the target's
  // velocity.
  inline const double GetTargetVel() const {return cur_target_vel_; }

  // Return the isam2 full backsubstitution results.
  const gtsam::Values CalculateFullTrajecory();

  // Return the approximate covariance of the chaser's pose.
  const Eigen::MatrixXd GetChaserPoseCovariance();

  // Return the estimated covariance for the target's pose.
  const Eigen::MatrixXd GetTargetPoseCovariance();

  // Return the estimated covariance for the target's position.
  const Eigen::MatrixXd GetTargetPosCovariance();

  // Return the estimated covariance for the target's position.
  const Eigen::MatrixXd GetTargetHeadCovariance();

  // Return the most recent estimated relative pose from the chaser to the
  // target, expressed in the chaser's frame.
  const Eigen::Affine3d GetRelativePoseEstimate();

  // Return the estimated covariance for the relative pose btwn the chaser and
  // the target, defined at the target.
  const Eigen::MatrixXd GetRelativePoseCovariance();

  // Uses the latest value of the heading and the position of the
  // target to return the pose of the target in the world frame.
  Eigen::Affine3d GetOptimizedTargetPose();

  // Initializes the graph with the input prior pose.
  int AddChaserPriorPose(
      const Eigen::Affine3d& prior_chaser_pose = Eigen::Affine3d::Identity());

  // Initializes the target's motion model variables in the graph.
  void AddTargetPriors(const Eigen::Vector3d& usbl_position,
                        double prior_target_velocity,
                        double target_heading);

  // Add relative USBL measurement to the graph. The USBL measurement
  // will represent our keyframe (will optimize the graph after adding)
  // until we get visual pose constraints. Returns true if keyframe was
  // optimized.
  bool AddUsblMeasurement(const Eigen::Vector3d& position, double delta_time);

  // Updates dead-reckoned estimate (from lolo's nav filter) and the current
  // slam estimate if the motion model has not been initialized yet.
  void AddDeadReckoningMeasurement(const Eigen::Affine3d& dr_pose,
                                   const Eigen::Vector3d& dr_velocity,
                                   const Eigen::MatrixXd& pose_covariance,
                                   const Eigen::Matrix3d& vel_covariance);

  // Accumulates the delta pose (in the chaser frame) measurements provided by
  // the chaser's INS; propagates the latest optimized state through
  // pose composition.
  void AddOdometryMeasurement(const Eigen::Affine3d& odom_pose,
                              const Eigen::Vector3d& odom_velocity,
                              const Eigen::MatrixXd& pose_covariance,
                              const Eigen::Matrix3d& vel_covariance);

  // Adds a relative pose constraint between the chaser and the target's
  // light-beacon center. Return true if the keyframe was optimized.
  bool AddOpticalMeasurement(const Eigen::Affine3d& optical_pose,
                            const Eigen::MatrixXd& optical_pose_cov,
                            double delta_time);

 private:
  // Configuration struct.
  const Config config_;

  // Initialize the noise models using specified parameters.
  void SetupNoiseModels();

  // Setup the extrinsic calibrations for both the target and the chaser.
  void SetupExtrinsics();

  // Setup iSAM's optimization and inference parameters.
  void SetupiSAM();

  // Setup IMU preintegrated measurements.
  //void SetupOdometers();

  // Bayes tree for incremental smoothing and mapping.
  gtsam::ISAM2 isam2_;

  // Factor graph for new observations.
  gtsam::NonlinearFactorGraph graph_;

  // Values to capture initial estimates for the new nodes.
  gtsam::Values initial_estimates_;

  // Variable for keeping track of the number of keyframes in the graph. Counter
  // should be increased at the end of the keyframed function.
  int cur_frame_ = 0;
  // Counter for keeping track of the current dynamics frame number. Only
  // used for velocities and heading. Value is increased everytime the
  // representation of the target is reduced from SE(3) to R3.
  int cur_dynamics_frame_ = 0;

  // Chaser values.

  // Odometry noise from INS.
  gtsam::noiseModel::Gaussian::shared_ptr odometer_noise_;

  // Noise model for the prior factor's measurement information.
  // The information comes from the chaser's INS dead-reckoning.
  gtsam::noiseModel::Gaussian::shared_ptr prior_pose_noise_;

  // Holds the most recent chaser pose in the world frame.
  gtsam::Pose3 cur_pose_estimate_;
  // Hold the mose recent chaser pose covariance wrt the body frame, at the current linearization point.
  gtsam::Matrix6 cur_pose_estimate_cov_ = gtsam::Z_6x6;
  // Contains the INS' dead reckoned pose wrt the world frame.
  gtsam::Pose3 dead_reckoning_ = gtsam::Pose3();
  // Contains the hitherto accumulated delta pose of the chaser.
  gtsam::Pose3 odometer_pose_ = gtsam::Pose3();
  // Contains the accumulated delta pose at the time of the latest USBL measurement.
  // Used for the reduced target's motion keyframe (SE(3) -> R3).
  gtsam::Pose3 reduced_keyframe_odometry_ = gtsam::Pose3();
  gtsam::noiseModel::Gaussian::shared_ptr reduced_keyframe_odom_noise_;
  // Contains the relevant USBL measurement required in case AddReducedKeyframe
  // is called.
  gtsam::Point3 reduced_keyframe_usbl_;

  // Chaser's camera extrinsics.
  gtsam::Pose3 C_tfm_cam_ = gtsam::Pose3();

  // Target values.

  // Prior noise models for the target's position.
  gtsam::noiseModel::Diagonal::shared_ptr prior_target_pos_noise_;
  // Prior noise for the target's velocity.
  gtsam::noiseModel::Diagonal::shared_ptr prior_target_vel_noise_;
  // Prior noise for the target's heading angle.
  gtsam::noiseModel::Diagonal::shared_ptr prior_target_head_noise_;

  // Noise models for the target's 3DoF motion model.
  gtsam::noiseModel::Diagonal::shared_ptr motion_model_noise_;
  // Noise models for the target's 3DoF -> 6DoF increased motion model.
  gtsam::noiseModel::Diagonal::shared_ptr increased_motion_noise_;
  // Noise model for the targets's 6DoF motion model.
  gtsam::noiseModel::Diagonal::shared_ptr target_pose_between_noise_;
  // Noise model for the target's USBL -> light beacons' extrinsic calibration.
  //gtsam::noiseModel::Isotropic::shared_ptr target_extrinsics_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr target_extrinsics_noise_;

  // Extrinsic calibration Lights -> USBL (target).
  gtsam::Pose3 L_tfm_T_ = gtsam::Pose3();
  // Extrinsic calibration USBL -> Lights (target), i.e. inverse of the above.
  gtsam::Pose3 T_tfm_L_ = gtsam::Pose3();

  // Latest optimized target position.
  gtsam::Point3 cur_target_pos_ = gtsam::Point3();
  // Latest optimized target position's covariance wrt to the world frame.
  gtsam::Matrix3 cur_target_pos_cov_ = gtsam::Z_3x3;
  // Latest optimized target pose.
  gtsam::Pose3 cur_target_pose_ = gtsam::Pose3();
  // Latest optimized target pose covariance wrt to the target's body frame, at
  // the current linearization point.
  gtsam::Matrix6 cur_target_pose_cov_ = gtsam::Z_6x6;
  // Latest optimized target velocity.
  double cur_target_vel_ = 0.0;
  // Latest optimized target velocity variance.
  gtsam::Matrix cur_target_vel_var_ = gtsam::Z_1x1;
  // Latest optimized target heading.
  double cur_target_head_ = 0.0;
  // Latest optimized target heading variance.
  gtsam::Matrix cur_target_head_var_ = gtsam::Z_1x1;

  // Flags.

  // Flag for checking whether the factor graph has been initialized.
  bool fg_init_ = false;
  // Flag for checking whether the target's prior has been initialized.
  bool is_target_init_ = false;
  // Flag for checking whether the chaser's prior has been added.
  bool is_chaser_init_ = false;
  // Flag for checking whether the target is in vision.
  bool is_target_visible_ = false;
  // Flag fopr checking whether the usbl is the only factor in the graph.
  bool is_usbl_alone_ = false;
};

}  // namespace dockslam

#endif  // DOCK_SLAM_GRAPH_MANAGER_H_
