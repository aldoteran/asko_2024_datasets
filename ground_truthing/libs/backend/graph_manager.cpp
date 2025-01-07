
#include "graph_manager.h"

namespace dockslam{

GraphManager::GraphManager(const Config &config) : config_(config) {
    SetupISAM2();
    SetupImuOdometer();
    SetupNoises();
    SetupExtrinsics();
}

GraphManager::~GraphManager(){
    // Save csv file.
    const std::string filename =
        "/home/aldoteran/docking_ws/src/asko_2024_datasets/ground_truthing/"
        "raw_keyframe_data.csv";
    std::cout << "Saving raw keframe data in " << filename << std::endl;
    csv_utils::DataToCsvFile(csvdata_, filename);
}

void GraphManager::SetupISAM2() {
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = 0.1;
  isam2_ = gtsam::ISAM2(params);
}

void GraphManager::SetupImuOdometer() {
  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> p =
      gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedD(-9.81);

  // Constant imu bias.
  imu_bias_ = gtsam::imuBias::ConstantBias(config_.init_accel_bias,
                                           config_.init_omega_bias);

  // These parameters are explained here: https://gtsam.org/ddoxygen/a03439.html
  const Eigen::DiagonalMatrix<double, 3> accel_cov(
      pow(config_.imu_accel_noise_stddev(0), 2),
      pow(config_.imu_accel_noise_stddev(1), 2),
      pow(config_.imu_accel_noise_stddev(2), 2));
  p->accelerometerCovariance = accel_cov;
  const Eigen::DiagonalMatrix<double, 3> omega_cov(
      pow(config_.imu_omega_noise_stddev(0), 2),
      pow(config_.imu_omega_noise_stddev(1), 2),
      pow(config_.imu_omega_noise_stddev(2), 2));
  p->gyroscopeCovariance = omega_cov;
  const Eigen::DiagonalMatrix<double, 3> accel_bias_cov(
      pow(config_.imu_accel_bias_stddev(0), 2),
      pow(config_.imu_accel_bias_stddev(1), 2),
      pow(config_.imu_accel_bias_stddev(2), 2));
  p->biasAccCovariance = accel_bias_cov;
  const Eigen::DiagonalMatrix<double, 3> omega_bias_cov(
      pow(config_.imu_omega_bias_stddev(0), 2),
      pow(config_.imu_omega_bias_stddev(1), 2),
      pow(config_.imu_omega_bias_stddev(2), 2));
  p->biasOmegaCovariance = omega_bias_cov;
  // Rule of thumb.
  p->integrationCovariance = gtsam::Matrix33::Identity(3, 3) * 1e-8;
  p->biasAccOmegaInt = gtsam::Matrix::Identity(6, 6) * 1e-5;

  // Instantiate both odometers.
  odometer_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(
      p, imu_bias_);

   std::cout << "Odometer is set up!" << std::endl;
}

void GraphManager::SetupExtrinsics() {
  // Target extrinsic calibration (SBG to center of light beacons).
  t_tfm_fid_ = gtsam::Pose3(
      gtsam::Rot3::Quaternion(config_.target_fiducials_extrinsics(3),
                              config_.target_fiducials_extrinsics(0),
                              config_.target_fiducials_extrinsics(1),
                              config_.target_fiducials_extrinsics(2)),
      gtsam::Point3(config_.target_fiducials_extrinsics(4),
                    config_.target_fiducials_extrinsics(5),
                    config_.target_fiducials_extrinsics(6)));
  // Inverse (light beacons to SBG).
  fid_tfm_t_ = t_tfm_fid_.inverse();
  // SBG to USBL acoustic center.
  t_tfm_tusbl_ =
      gtsam::Pose3(gtsam::Rot3::Ypr(config_.target_usbl_extrinsics(2),
                                    config_.target_usbl_extrinsics(1),
                                    config_.target_usbl_extrinsics(0)),
                   gtsam::Point3(config_.target_usbl_extrinsics(3),
                                 config_.target_usbl_extrinsics(4),
                                 config_.target_usbl_extrinsics(5)));

  // Chaser's camera's extrinsic calibration from base_link to camera_link.
  c_tfm_cam_ =
      gtsam::Pose3(gtsam::Rot3::Quaternion(config_.chaser_camera_extrinsics(3),
                                           config_.chaser_camera_extrinsics(0),
                                           config_.chaser_camera_extrinsics(1),
                                           config_.chaser_camera_extrinsics(2)),
                   gtsam::Point3(config_.chaser_camera_extrinsics(4),
                                 config_.chaser_camera_extrinsics(5),
                                 config_.chaser_camera_extrinsics(6)));
  // Inverse, camera to baser_link.
  cam_tfm_c_ = c_tfm_cam_.inverse();
  // Chaser base_link to chaser's USBL.
  c_tfm_cusbl_ =
      gtsam::Pose3(gtsam::Rot3::Ypr(config_.chaser_usbl_extrinsics(2),
                                    config_.chaser_usbl_extrinsics(1),
                                    config_.chaser_usbl_extrinsics(0)),
                   gtsam::Point3(config_.chaser_usbl_extrinsics(3),
                                 config_.chaser_usbl_extrinsics(4),
                                 config_.chaser_usbl_extrinsics(5)));
}

void GraphManager::SetupNoises() {
  // Usbl position noise.
  target_usbl_noise_ =
      gtsam::noiseModel::Diagonal::Sigmas(config_.usbl_noise_stddev);
  if (config_.optimize_chaser){
      // Adds a large uncertainty on the rotational stddevs since we're
      // using a Pose3 between factor.
      gtsam::Vector usbl_sigmas(6);
      usbl_sigmas << 1e9, 1e9, 1e9, config_.usbl_noise_stddev(0),
          config_.usbl_noise_stddev(1), config_.usbl_noise_stddev(2);
      target_usbl_between_noise_ =
          gtsam::noiseModel::Diagonal::Sigmas(usbl_sigmas);

      // Very small prior noise on the chaser to anchor the graph.
      gtsam::Vector chaser_sigmas(6);
      chaser_sigmas << 0.0000001, 0.000001, 0.000001, 0.000001, 0.000001,
          0.000001;
      chaser_prior_noise_ = gtsam::noiseModel::Diagonal::Sigmas(chaser_sigmas);
  }

  // IMU bias random walk.
  gtsam::Vector imu_bias_sigmas(6);
  imu_bias_sigmas << config_.imu_accel_bias_stddev(0),
      config_.imu_accel_bias_stddev(1), config_.imu_accel_bias_stddev(2),
      config_.imu_omega_bias_stddev(0), config_.imu_omega_bias_stddev(1),
      config_.imu_omega_bias_stddev(2);
  imu_bias_noise_ = gtsam::noiseModel::Diagonal::Sigmas(imu_bias_sigmas);

  gtsam::Vector init_imu_bias_sigmas(6);
  init_imu_bias_sigmas << config_.init_accel_bias_stddev,
      config_.init_accel_bias_stddev, config_.init_accel_bias_stddev,
      config_.init_omega_bias_stddev, config_.init_omega_bias_stddev,
      config_.init_omega_bias_stddev;
  init_imu_bias_noise_ = gtsam::noiseModel::Diagonal::Sigmas(init_imu_bias_sigmas);

  // Optical measurement noise model.
  gtsam::Vector optical_meas_sigmas(6);
  optical_meas_sigmas << config_.optical_rot_stddev,
      config_.optical_rot_stddev, config_.optical_rot_stddev,
      config_.optical_trans_stddev, config_.optical_trans_stddev,
      config_.optical_trans_stddev;
  optical_meas_noise_ = gtsam::noiseModel::Diagonal::Sigmas(optical_meas_sigmas);

}

void GraphManager::SetupPriors() {
  if (!hdt_ready_) {
    return;
  }
  // Initial uncertainties from config file.
  gtsam::noiseModel::Diagonal::shared_ptr target_init_pose_noise =
      gtsam::noiseModel::Diagonal::Sigmas(config_.target_init_pose_stddev);
  gtsam::noiseModel::Diagonal::shared_ptr target_init_vel_noise =
      gtsam::noiseModel::Diagonal::Sigmas(config_.target_init_vel_stddev);

  const gtsam::Rot3 init_rot = gtsam::Rot3::Rz(true_heading_) *
                               gtsam::Rot3::Ry(true_pitch_) *
                               gtsam::Rot3::Rx(M_PI);
  gtsam::Rot3::Rz(true_heading_);
  const gtsam::Point3 init_pos = target_odom_pose_.translation();
  target_init_pose_ = gtsam::Pose3(init_rot, init_pos);

  // Initialize the values for the target's state.
  target_global_pose_ = target_init_pose_;
  target_vel_ = gtsam::Vector3(0., 0., 0.);

  initial_estimates_.insert(X(0), target_global_pose_);
  initial_estimates_.insert(V(0), target_vel_);
  initial_estimates_.insert(B(0), imu_bias_);

  graph_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      X(0), target_global_pose_, target_init_pose_noise);
  graph_.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(
      V(0), target_vel_, target_init_vel_noise);
  graph_.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
      B(0), imu_bias_, init_imu_bias_noise_);
  if (config_.optimize_chaser) {
    initial_estimates_.insert(C(0), chaser_measured_pose_);
    graph_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
        C(0), chaser_measured_pose_, chaser_prior_noise_);
    chaser_previous_pose_ = chaser_measured_pose_;
    chaser_previous_noise_ = chaser_measured_noise_;
  }

  // Prior timestamp at zero.
  timestamps_[cur_frame_] = 0.0;
  // Start counting frames.
  cur_frame_ = 1;
  is_graph_init = true;

  std::cout << "Priors are done!" << std::endl;
}

void GraphManager::AddTargetGlobalState(
    const Eigen::Affine3d &pose, const Eigen::Vector3d &vel,
    const Eigen::Matrix<double, 6, 6> &pose_cov, double stamp) {
  target_odom_pose_ = gtsam::Pose3(pose.matrix());
  target_odom_vel_ = gtsam::Vector3(vel);
  target_odom_pose_noise_ =
      gtsam::noiseModel::Gaussian::Covariance(pose_cov.matrix());

  if (!is_target_init){
    is_target_init = true;
  }
}

void GraphManager::AddChaserGlobalPose(const Eigen::Affine3d &pose,
                                       const Eigen::Matrix<double, 6, 6> &cov,
                                       double stamp) {
  chaser_measured_pose_ = gtsam::Pose3(pose.matrix());
  // FIXME: add a covariance scaling factor.
  chaser_measured_noise_ =
      gtsam::noiseModel::Gaussian::Covariance(cov.matrix() * 0.00005);
  chaser_pose_stamp_ = stamp;

  if (!is_chaser_init) {
    is_chaser_init = true;
  }
}

void GraphManager::UpdateISAM2(){
  // Print before resetting.
  graph_.print("------- KEYFRAME ------");
  // Update ISAM2.
  isam2_.update(graph_, initial_estimates_);
  for (int k = 0; k < 2; k++) {
    isam2_.update();
  }

  // Full back-substitution at every keyframe.
  gtsam::Values results = isam2_.calculateBestEstimate();
  // Update all current values.
  target_global_pose_ = results.at<gtsam::Pose3>(X(cur_frame_));
  target_vel_ = results.at<gtsam::Vector3>(V(cur_frame_));
  imu_bias_ = results.at<gtsam::imuBias::ConstantBias>(B(cur_frame_));
  // Get marginal covariance of target's current pose.
  target_marginal_noise_ = gtsam::noiseModel::Gaussian::Covariance(
      isam2_.marginalCovariance(X(cur_frame_)));

  if(config_.optimize_chaser){
      chaser_optimized_pose_ = results.at<gtsam::Pose3>(C(cur_frame_));
      chaser_marginal_noise_ = gtsam::noiseModel::Gaussian::Covariance(
          isam2_.marginalCovariance(C(cur_frame_)));
  }

  // Reset factor graph.
  graph_.resize(0);
  // Reset odometer integration.
  odometer_->resetIntegrationAndSetBias(imu_bias_);
  // Reset initial estimates.
  initial_estimates_.clear();

  std::cout << "(GraphManager) Optimized the graph!!" << std::endl;
  std::cout << "(GraphManager) Target pose: \n" << target_global_pose_ << "\n";
  std::cout << "(GraphManager) Target marginals noise: \n"
            << target_marginal_noise_->covariance() << "\n";
  if (config_.optimize_chaser) {
    std::cout << "(GraphManager) Chaser pose: \n"
              << chaser_optimized_pose_ << "\n";
    std::cout << "(GraphManager) Chaser marginals noise: \n"
              << chaser_marginal_noise_->covariance() << "\n";
  }
}

void GraphManager::AddChaserRelativeOpticalPose(
    const Eigen::Affine3d &pose, const Eigen::Matrix<double, 6, 6> &cov,
    double stamp) {
  // Ok, this is the measurement from Lolo's camera to the center of the light
  // fiducials, which means that we have to add the lolo->camera extrinsic
  // calibration and the lights->boat extrinsic calibration as well. To add it
  // as a global factor, we must also tranform the resulting relative pose so
  // that it is expressed in the map frame. At the end, we want: W_tfm_T =
  // W_tfm_C (+) C_tfm_cam (+) cam_tfm_lights (+) lights_tfm_T.
  if (!is_graph_init) {
    std::cout << "(AddTargetGps) Graph not initialized yet!" << std::endl;
    return;
  }

  if (!is_chaser_init) {
    std::cout << "Chaser not initialized yet." << std::endl;
    return;
  }

  // Check the timestamp difference. Half a second is too much.
  if (std::abs(stamp - chaser_pose_stamp_) > 0.5) {
    std::cout << "Optical and global pose are unsynced." << std::endl;
    return;
  }

  // Simple outlier rejection, if Lolo is not underwater, ignore.
  if (chaser_measured_pose_.z() > -0.5) {
    std::cout << "(AddOptical) Lolo is not underwater, prolly an outlier."
              << std::endl;
    return;
  }

  if (!IsImuReady()){
      std::cout << "IMU factor not ready! Skipping keyframe." << std::endl;
      return;
  }
  // Measurement to Pose3.
  const gtsam::Pose3 cam_tfm_fid(pose.matrix());
  // Measurement noise.
  gtsam::noiseModel::Gaussian::shared_ptr meas_noise =
      gtsam::noiseModel::Gaussian::Covariance(cov.matrix());

  // The optical factor can be added either as a betweenfactor (full SLAM)
  // or a global optical factor(PoseSLAM).
  if (config_.optimize_chaser){
      // Measured relative pose.
      const gtsam::Pose3 c_tfm_t = c_tfm_cam_ * cam_tfm_fid * fid_tfm_t_;
      // BetweenFactor from chasers current pose to target's current pose.
      graph_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          C(cur_frame_), X(cur_frame_), c_tfm_t, optical_meas_noise_);
      // Add a between factor and initial conditions from the previous to the
      // current chaser frame using the chaser's INS measurements.
      AddChaserBetweenFactor(stamp);

      std::cout << "Added optical pose:\n" << c_tfm_t << "\n";
      std::cout << "with noise model: \n"
                << optical_meas_noise_->covariance() << "\n";
  } else{
      // Add a global prior optical factor instead.
      // Compose mean for factor in the world frame.
      const gtsam::Pose3 w_tfm_t =
          chaser_measured_pose_ * c_tfm_cam_ * cam_tfm_fid * fid_tfm_t_;
      // Compute factor noise.
      gtsam::noiseModel::Gaussian::shared_ptr factor_noise;
      ComputeOpticalGlobalFactorNoise(
          cam_tfm_fid, w_tfm_t, chaser_measured_pose_, factor_noise, c_tfm_cam_,
          fid_tfm_t_,
          /*meas_noise*/ optical_meas_noise_, chaser_measured_noise_);
      // Add factor to graph.
      graph_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
          X(cur_frame_), w_tfm_t, factor_noise);
  }

  // Add a between factor and initial conditions from the previous to the
  // current frame using the imu preintegrated.
  AddImuBetweenFactor();

  // Add new keyframe.
  UpdateISAM2();

  // Bookkeeping.
  timestamps_[cur_frame_] = stamp;
  optical_frames_.push_back(cur_frame_);
  cur_frame_++;

  // Logging.
  csv_utils::AppendOpticalKeyframe(csvdata_, cam_tfm_fid, chaser_measured_pose_,
                                   target_odom_pose_, stamp, /*as_quat=*/true);
}

void GraphManager::AddTargetUsblFix(const Eigen::Vector3d &pos, double stamp) {
  // The boat's usbl fix is expressed in the frame of reference of the boat's
  // usbl and measures a distance to lolo's usbl transponder. Since we need the
  // boat's rotation wrt the world/map frame (W_rot_T), which we are estimating,
  // to express it in that frame (so we can add it as a uunary factor on the
  // boat's state), we have to add our own custom factor.
  if (/*config_.no_usbl*/ true) {
    if (!is_chaser_init) {
      std::cout << "Chaser not initialized yet." << std::endl;
      return;
    }

    // Check the timestamp difference. Half a second is too much.
    if (std::abs(stamp - chaser_pose_stamp_) > 0.5) {
      std::cout << "USBL and global pose are unsynced." << std::endl;
      return;
    }

    if (!IsImuReady()) {
      std::cout << "IMU factor not ready! Skipping keyframe." << std::endl;
      return;
    }
    // Best guess for initial conditions for the target.
    const gtsam::NavState prev_state(target_global_pose_, target_vel_);
    const gtsam::NavState target_cur_state =
        odometer_->predict(prev_state, imu_bias_);
    const gtsam::Pose3 target_cur_pose = target_cur_state.pose();

    // Add the USBL measurement as a relative position measurement from
    // the target to the chaser if the chaser is to be optimized, otherwise
    // add a global prior factor on the target's position.
    if (config_.optimize_chaser) {
      const gtsam::Rot3 t_rot_c = target_cur_pose.rotation().inverse() *
                                  chaser_measured_pose_.rotation();
      const gtsam::Point3 t_trans_c_t = t_tfm_tusbl_.translation() +
                                        t_tfm_tusbl_.rotation() * pos -
                                        t_rot_c * c_tfm_cusbl_.translation();
      graph_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          X(cur_frame_), C(cur_frame_),
          gtsam::Pose3(gtsam::Rot3(), t_trans_c_t), target_usbl_between_noise_);
    } else {
      // Measurement struct.
      const UsblMeasurement meas = {gtsam::Point3(pos), chaser_measured_pose_,
                                    c_tfm_cusbl_, t_tfm_tusbl_};
      // Noise model for factor.
      gtsam::noiseModel::Gaussian::shared_ptr factor_noise;
      ComputeUsblGlobalFactorNoise(meas, target_cur_pose, factor_noise,
                                   target_usbl_noise_, chaser_measured_noise_,
                                   target_marginal_noise_);
      // Add custom factor to graph.
      graph_.emplace_shared<gtsam::UsblGlobalFactor>(X(cur_frame_), meas,
                                                     factor_noise);
    }

    // Add a between factor and initial conditions from the previous to the
    // current frame using the imu preintegrated.
    AddImuBetweenFactor();

    if (config_.optimize_chaser) {
      // Add a between factor and initial conditions from the previous to the
      // current chaser frame using the chaser's INS measurements.
      AddChaserBetweenFactor(stamp);
    }

    // Add new keyframe.
    UpdateISAM2();

    // Bookkeeping.
    timestamps_[cur_frame_] = stamp;
    usbl_frames_.push_back(cur_frame_);
    cur_frame_++;
  }

  csv_utils::AppendUsblKeyframe(csvdata_, gtsam::Point3(pos),
                                chaser_measured_pose_, target_odom_pose_,
                                stamp, /*as_quat=*/true);
}

void GraphManager::AddTargetImu(const Eigen::Vector3d &acc,
                                const Eigen::Vector3d &omega, double stamp) {
  if (!is_target_init) {
    std::cout << "Target not initialized yet!" << std::endl;
    return;
  }
  odometer_->integrateMeasurement(acc, omega, 1.0/config_.imu_frequency);
}

void GraphManager::AddTargetGps(const Eigen::Vector3d &gps_pos,
                                const Eigen::Vector3d &stddev, double stamp) {
  if (!is_graph_init) {
    std::cout << "(AddTargetGps) Graph not initialized yet!" << std::endl;
    return;
  }

  if (!IsImuReady()){
      std::cout << "IMU factor not ready! Skipping keyframe." << std::endl;
      return;
  }

  const gtsam::Point3 pos(gps_pos);
  gtsam::noiseModel::Diagonal::shared_ptr noise =
      gtsam::noiseModel::Diagonal::Sigmas(stddev*5.0);

  graph_.emplace_shared<gtsam::GPSFactor>(X(cur_frame_), pos, noise);
  gps_in_ = true;

  // FIXME.
  if (true) {
    AddImuBetweenFactor();

    if (config_.optimize_chaser) {
      // Add a between factor and initial conditions from the previous to the
      // current chaser frame using the chaser's INS measurements.
      AddChaserBetweenFactor(stamp);
    }

    UpdateISAM2();
    hdt_in_ = false;
    gps_in_ = false;
    // Bookkeeping.
    timestamps_[cur_frame_] = stamp;
    cur_frame_++;
  }

}

void GraphManager::AddTargetHdt(double heading, double heading_stddev,
                                double pitch, double pitch_stddev,
                                double stamp) {

  // Store for initialization.
  true_heading_ = heading;
  true_heading_noise_ = heading_stddev;
  true_pitch_ = pitch;
  true_pitch_noise_ = pitch_stddev;
  hdt_ready_ = true;
  hdt_in_ = true;
  return;

  if (!is_graph_init) {
    std::cout << "(AddTargetHdt) Graph not initialized yet!" << std::endl;
    return;
  }

  if (!IsImuReady()){
      std::cout << "IMU factor not ready! Skipping keyframe." << std::endl;
      return;
  }

  const gtsam::Vector2 meas(pitch, heading);
  const gtsam::Vector2 sigmas(pitch_stddev, heading_stddev);
  gtsam::noiseModel::Diagonal::shared_ptr noise =
      gtsam::noiseModel::Diagonal::Sigmas(sigmas);

  // We're gonna use a PartialPriorFactor for this one.
  // The mask contains the indices of the DOF we are constraining,
  // the indices are wrt the Logmap() of the variable node we're constraining,
  // i.e. Pose3, so we're looking at [Rx, Ry, Rz, X, Y, Z]. Since we want
  // to constrain pitch and yaw, we use indices 1 and 2.
  std::vector<size_t> mask = {1, 2};
  // FIXME: PartialPriorFactor doesn't seem to work!
  graph_.emplace_shared<gtsam::PartialPriorFactor<gtsam::Pose3>>(
      X(cur_frame_), mask, meas, noise);
  hdt_in_ = true;

  // Add a between factor and initial conditions from the previous to the
  // current frame using the imu preintegrated.
  if(gps_in_){
    AddImuBetweenFactor();
    if (config_.optimize_chaser) {
      // Add a between factor and initial conditions from the previous to the
      // current chaser frame using the chaser's INS measurements.
      AddChaserBetweenFactor(stamp);
    }
    UpdateISAM2();
    gps_in_ = false;
    hdt_in_ = false;
    // Bookkeeping.
    timestamps_[cur_frame_] = stamp;
    cur_frame_++;
  }
}

bool GraphManager::IsImuReady() {
  if (odometer_->deltaVij() == gtsam::Vector3(0, 0, 0)) {
    std::cout << "No IMU measurements yet!" << std::endl;
    return false;
  }
  return true;
}

bool GraphManager::IsGraphInit() {
  return is_graph_init;
}

void GraphManager::AddImuBetweenFactor(){
  if (!is_graph_init) {
    std::cout << "(AddImuBetween) Graph not initialized yet!" << std::endl;
    return;
  }

  const gtsam::NavState prev_state(target_global_pose_, target_vel_);
  const gtsam::NavState target_state = odometer_->predict(prev_state, imu_bias_);
  target_global_pose_ = target_state.pose();
  target_vel_ = target_state.velocity();

  // Add the computed inital conditions.
  initial_estimates_.insert(X(cur_frame_), target_state.pose());
  initial_estimates_.insert(V(cur_frame_), target_state.velocity());
  initial_estimates_.insert(B(cur_frame_), imu_bias_);

  // FIXME: Add a standalone calibration routine.
  // Uncomment for IMU bias calibration. Assumes zero movement to estimate
  // the IMU and gyro biases.
  //initial_estimates_.insert(X(cur_frame_), target_init_pose_);
  //initial_estimates_.insert(V(cur_frame_), gtsam::Vector3(0., 0., 0.));
  //initial_estimates_.insert(B(cur_frame_), imu_bias_);
  //graph_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      //X(cur_frame_), target_init_pose_,
      //gtsam::noiseModel::Isotropic::Sigma(6, 0.0001));

  // Since the IMU measurements are pretty shit, we'll constrain the attitude
  // of the target using the GPS HDT angles and a constant roll.
  const gtsam::Rot3 target_attitude = gtsam::Rot3::Rz(true_heading_) *
                                      gtsam::Rot3::Ry(true_pitch_) *
                                      gtsam::Rot3::Rx(M_PI);
  // Adding the right translation but will try to avoid double counting it by
  // adding a large uncertainty to it. +- 5 degree uncertainty in the roll.
  gtsam::Vector target_attitude_diag(6);
  // FIXME: add uncertainty as parameter in config file.
  target_attitude_diag << 0.0872, true_pitch_noise_, true_heading_noise_, 1e6,
      1e6, 1e6;
  graph_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      X(cur_frame_),
      gtsam::Pose3(target_attitude, target_global_pose_.translation()),
      gtsam::noiseModel::Diagonal::Sigmas(target_attitude_diag));

  // Add IMU factor.
  graph_.emplace_shared<gtsam::CombinedImuFactor>(
      X(cur_frame_ - 1), V(cur_frame_ - 1), X(cur_frame_), V(cur_frame_),
      B(cur_frame_ - 1), B(cur_frame_), *odometer_);
}

void GraphManager::AddChaserBetweenFactor(double stamp){
  if (std::abs(stamp - chaser_pose_stamp_) > 0.5) {
    std::cout << "GPS stamp and chaser pose not synced!\n";
  }

  // Compute the relative pose between the most recent measured
  // pose of the chaser and the last keyframe's measured pose.
  const gtsam::Pose3 ci_tfm_cj =
      chaser_previous_pose_.inverse() * chaser_measured_pose_;
  // Compute the noise for the factor.
  gtsam::noiseModel::Gaussian::shared_ptr factor_noise;
  // FIXME: we're computing the relative noise without taking into account
  // the correlation between pose i and pose j, so the result is a covariance
  // which is bigger than that wrt either poses (which is incorrect). We
  // need to consider the correlation to properly represent this covariance
  // (as in Mangelson's paper).
  ComputeRelativeTfmNoise(chaser_previous_pose_, chaser_measured_pose_,
                          chaser_previous_noise_, chaser_measured_noise_,
                          factor_noise);

  std::cout << "prev chaser pose:\n" << chaser_previous_pose_ << "\n";
  std::cout << "cur chaser pose:\n" << chaser_measured_pose_ << "\n";
  std::cout << "prev pose * delta pose" << chaser_previous_pose_ * ci_tfm_cj
            << "\n";

  // Initial estimates are going to be the previously optimized pose
  // composed by the delta T measured by the INS.
  initial_estimates_.insert(
      C(cur_frame_),
      /*FIXME: chaser_optimized_pose_*/ chaser_previous_pose_ * ci_tfm_cj);

  // Add between factor.
  graph_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      C(cur_frame_ - 1), C(cur_frame_), ci_tfm_cj,
      /*FIXME:factor_noise*/ chaser_measured_noise_);
  // FIXME: Let's constrain the shit out of Lolo's depth.
  graph_.emplace_shared<gtsam::DepthFactor>(
      C(cur_frame_), chaser_measured_pose_.z(),
      gtsam::noiseModel::Isotropic::Sigma(1, 0.000001));

  chaser_previous_pose_ = chaser_measured_pose_;
  chaser_previous_noise_ = chaser_measured_noise_;
}

void GraphManager::SaveGraph(std::ofstream &filename) {
  graph_.saveGraph(filename);
}

gtsam::Values GraphManager::OptimizeOnce() {
  return gtsam::LevenbergMarquardtOptimizer(graph_, initial_estimates_)
      .optimize();
}

gtsam::Values GraphManager::Optimize4Real() {
  for (size_t i = 0; i < 5; i++) {
    initial_estimates_ =
        gtsam::LevenbergMarquardtOptimizer(graph_, initial_estimates_)
            .optimize();
  }
  return initial_estimates_;
}

void GraphManager::Print() {
  graph_.print("-----------FACTOR GRAPH----------\n");
}

void GraphManager::PrintInitialEstimates() {
  initial_estimates_.print("---------INITIAL ESTIMATES----------\n");
}

gtsam::Values GraphManager::PrintISAM2Results(const std::string &path_to_data){
    gtsam::Values results = isam2_.calculateBestEstimate();
    results.print("------------- ISAM2 RESULTS -----------\n");

    std::cout << " --------- MARGINAL COVARIANCE --------\n";
    std::cout << "imu bias cov: \n"
              << isam2_.marginalCovariance(B(cur_frame_ - 1)) << "\n";
    std::cout << "target pose cov: \n"
              << isam2_.marginalCovariance(X(cur_frame_ - 1)) << "\n";
    std::cout << "target vel cov: \n"
              << isam2_.marginalCovariance(V(cur_frame_ - 1)) << "\n";

    csv_utils::ValuesToCsvFile(results, timestamps_, path_to_data,
                               config_.optimize_chaser, optical_frames_,
                               usbl_frames_);

    return results;
}

std::map<int, double> GraphManager::GetTimestamps() { return timestamps_; }

} // namespace dockslam.
