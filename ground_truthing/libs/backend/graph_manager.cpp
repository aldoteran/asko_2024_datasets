
#include "graph_manager.h"

namespace dockslam{

GraphManager::GraphManager(const Config &config) : config_(config) {
    SetupImuOdometer();
    SetupNoises();
    SetupExtrinsics();
}

GraphManager::~GraphManager(){
    // Save csv file.
    std::cout << "Calling destructor" << std::endl;
    const std::string filename =
        "/home/aldoteran/docking_ws/src/asko_2024_datasets/ground_truthing/"
        "raw_keyframe_data.csv";
    csv_utils::DataToCsvFile(csvdata_, filename);
}

void GraphManager::SetupImuOdometer() {
  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> p =
      gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedD();

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
  // FIXME: RzRyRx will not work.
  // Target extrinsic calibration (SBG to center of light beacons).
  target_fiducial_extr_ =
      gtsam::Pose3(gtsam::Rot3::RzRyRx(config_.target_fiducials_extrinsics(2),
                                       config_.target_fiducials_extrinsics(1),
                                       config_.target_fiducials_extrinsics(0)),
                   gtsam::Point3(config_.target_fiducials_extrinsics(3),
                                 config_.target_fiducials_extrinsics(4),
                                 config_.target_fiducials_extrinsics(5)));
  target_usbl_extr_ =
      gtsam::Pose3(gtsam::Rot3::RzRyRx(config_.target_usbl_extrinsics(2),
                                       config_.target_usbl_extrinsics(1),
                                       config_.target_usbl_extrinsics(0)),
                   gtsam::Point3(config_.target_usbl_extrinsics(3),
                                 config_.target_usbl_extrinsics(4),
                                 config_.target_usbl_extrinsics(5)));

  // Chaser's camera's extrinsic calibration from base_link to camera_link.
  chaser_camera_extr_ =
      gtsam::Pose3(gtsam::Rot3::RzRyRx(config_.chaser_camera_extrinsics(2),
                                       config_.chaser_camera_extrinsics(1),
                                       config_.chaser_camera_extrinsics(0)),
                   gtsam::Point3(config_.chaser_camera_extrinsics(3),
                                 config_.chaser_camera_extrinsics(4),
                                 config_.chaser_camera_extrinsics(5)));
  chaser_usbl_extr_ =
      gtsam::Pose3(gtsam::Rot3::RzRyRx(config_.chaser_usbl_extrinsics(2),
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

  // IMU bias random walk.
  gtsam::Vector imu_bias_sigmas(6);
  imu_bias_sigmas << config_.imu_accel_bias_stddev(0),
      config_.imu_accel_bias_stddev(1), config_.imu_accel_bias_stddev(2),
      config_.imu_omega_bias_stddev(0), config_.imu_omega_bias_stddev(1),
      config_.imu_omega_bias_stddev(2);
  imu_bias_noise_ = gtsam::noiseModel::Diagonal::Sigmas(imu_bias_sigmas);
}

void GraphManager::SetupPriors() {
  // Initial uncertainties from config file.
  gtsam::noiseModel::Diagonal::shared_ptr target_init_pose_noise =
      gtsam::noiseModel::Diagonal::Sigmas(config_.target_init_pose_stddev);
  gtsam::noiseModel::Diagonal::shared_ptr target_init_vel_noise =
      gtsam::noiseModel::Diagonal::Sigmas(config_.target_init_vel_stddev);

  // Initialize the values for the target's state.
  target_global_pose_ = target_odom_pose_;
  target_vel_ = gtsam::Vector3(0., 0., 0.);

  initial_estimates_.insert(X(0), target_odom_pose_);
  initial_estimates_.insert(V(0), target_vel_);
  initial_estimates_.insert(B(0), imu_bias_);

  graph_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      X(0), target_odom_pose_, target_init_pose_noise);
  graph_.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(
      V(0), target_vel_, target_init_vel_noise);
  graph_.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
      B(0), imu_bias_, imu_bias_noise_);

  cur_frame_ = 1;
  is_graph_init = true;

  std::cout << "Priors are done!" << std::endl;
}

void GraphManager::AddTargetGlobalState(
    const Eigen::Affine3d &pose, const Eigen::Vector3d &vel,
    const Eigen::Matrix<double, 6, 6> &pose_cov, double stamp) {

  target_odom_pose_ = gtsam::Pose3(pose.matrix());
  target_odom_vel_ = gtsam::Vector3(vel);

  target_pose_noise_ =
      gtsam::noiseModel::Gaussian::Covariance(pose_cov.matrix());

  if (!is_target_init){
    is_target_init = true;
  }

}

void GraphManager::AddChaserGlobalPose(const Eigen::Affine3d &pose,
                                       const Eigen::Matrix<double, 6, 6> &cov,
                                       double stamp) {
    chaser_global_pose_ = gtsam::Pose3(pose.matrix());
    chaser_global_noise_ = gtsam::noiseModel::Gaussian::Covariance(cov.matrix());
    chaser_pose_stamp_ = stamp;

    if (!is_chaser_init){
        is_chaser_init = true;
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
  if (chaser_global_pose_.z() > -0.5) {
    std::cout << "(AddOptical) Lolo is not underwater, prolly an outlier."
              << std::endl;
    return;
  }

  //std::cout << "Adding optical factor in frame " << cur_frame_
            //<< std::endl;

  // Compose to get W_tfm_T.
  const gtsam::Pose3 cam_tfm_lights(pose.matrix());
  const gtsam::Pose3 w_tfm_t =
      chaser_global_pose_ * chaser_camera_extr_ * cam_tfm_lights * target_fiducial_extr_;

  // TODO: compose covariances as well.
  const gtsam::Matrix6 covariance /* = bla bla bla */;
  gtsam::noiseModel::Gaussian::shared_ptr noise =
      gtsam::noiseModel::Gaussian::Covariance(cov.matrix());

  // Add a between factor and initial conditions from the previous to the
  // current frame using the imu preintegrated.
  AddImuBetweenFactor();

  graph_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(cur_frame_),
                                                          w_tfm_t, noise);

  cur_frame_++;

  csv_utils::AppendOpticalKeyframe(csvdata_, cam_tfm_lights,
                                   chaser_global_pose_, target_global_pose_,
                                   stamp);
}

void GraphManager::AddTargetUsblFix(const Eigen::Vector3d &pos, double stamp) {
  // The boat's usbl fix is expressed in the frame of reference of the boat's
  // usbl and measures a distance to lolo's usbl transponder. Since we need the
  // boat's rotation wrt the world/map frame (W_rot_T), which we are estimating,
  // to express it in that frame (so we can add it as a uunary factor on the
  // boat's state), we have to add our own custom factor.

  // Sanity checks.
  if (!is_chaser_init) {
    std::cout << "Chaser not initialized yet." << std::endl;
    return;
  }

  // Check the timestamp difference. Half a second is too much.
  if (std::abs(stamp - chaser_pose_stamp_) > 0.5) {
    std::cout << "USBL and global pose are unsynced." << std::endl;
    return;
  }

  const UsblMeasurement meas = {gtsam::Point3(pos), chaser_global_pose_,
                                chaser_usbl_extr_, target_usbl_extr_};

  // Add a between factor and initial conditions from the previous to the
  // current frame using the imu preintegrated.
  //AddImuBetweenFactor();

  // TODO: Add unary TargetUsblFactor.
  // graph_.emplace_shared<TargetUsblFactor>(X(cur_frame_), meas, usbl_noise_);

 //cur_frame_++;

  csv_utils::AppendUsblKeyframe(csvdata_, gtsam::Point3(pos),
                                chaser_global_pose_, target_global_pose_,
                                stamp);
}

void GraphManager::AddTargetImu(const Eigen::Vector3d &acc,
                                const Eigen::Vector3d &omega, double stamp) {

  if (!is_target_init) {
    std::cout << "Target not initialized yet!" << std::endl;
    return;
  }

  odometer_->integrateMeasurement(acc, omega, 1.0/config_.imu_frequency);

  prev_imu_stamp_ = stamp;
}

void GraphManager::AddTargetGps(const Eigen::Vector3d &gps_pos,
                                const Eigen::Vector3d &stddev, double stamp) {

  if (!is_graph_init) {
    std::cout << "(AddTargetGps) Graph not initialized yet!" << std::endl;
    return;
  }

  //std::cout << "Adding GPS factor in frame " << cur_frame_
            //<< std::endl;

  const gtsam::Point3 pos(gps_pos);
  gtsam::noiseModel::Diagonal::shared_ptr noise =
      gtsam::noiseModel::Diagonal::Sigmas(stddev);

  // Add a between factor and initial conditions from the previous to the
  // current frame using the imu preintegrated.
  AddImuBetweenFactor();

  graph_.emplace_shared<gtsam::GPSFactor>(X(cur_frame_), pos, noise);

  cur_frame_++;
}

void GraphManager::AddTargetHdt(double heading, double heading_stddev,
                                double pitch, double pitch_stddev,
                                double stamp) {

  if (!is_graph_init) {
    std::cout << "(AddTargetHdt) Graph not initialized yet!" << std::endl;
    return;
  }

  //std::cout << "Adding HDT PartialPriorFactor in frame " << cur_frame_
            //<< std::endl;

  const gtsam::Vector2 meas(pitch, heading);
  const gtsam::Vector2 sigmas(pitch_stddev, heading_stddev);
  gtsam::noiseModel::Diagonal::shared_ptr noise =
      gtsam::noiseModel::Diagonal::Sigmas(sigmas);

  // Add a between factor and initial conditions from the previous to the
  // current frame using the imu preintegrated.
  AddImuBetweenFactor();

  // We're gonna use a PartialPriorFactor for this one.
  // The mask contains the indices of the DOF we are constraining,
  // the indices are wrt the Logmap() of the variable node we're constraining,
  // i.e. Pose3, so we're looking at [Rx, Ry, Rz, X, Y, Z]. Since we want
  // to constrain pitch and yaw, we use indices 1 and 2.
  std::vector<size_t> mask = {1, 2};

  graph_.emplace_shared<gtsam::PartialPriorFactor<gtsam::Pose3>>(
      X(cur_frame_), mask, meas, noise);

  cur_frame_++;
}

void GraphManager::AddImuBetweenFactor(){

  if (!is_graph_init) {
    std::cout << "(AddImuBetween) Graph not initialized yet!" << std::endl;
    return;
  }

  //std::cout << "Adding IMU factor in frame " << cur_frame_
            //<< std::endl;

  const gtsam::NavState prev_state(target_global_pose_, target_vel_);
  const gtsam::NavState target_state = odometer_->predict(prev_state, imu_bias_);
  target_global_pose_ = target_state.pose();
  target_vel_ = target_state.velocity();

  // Add the computed inital conditions.
  initial_estimates_.insert(X(cur_frame_), target_state.pose());
  initial_estimates_.insert(V(cur_frame_), target_state.velocity());
  initial_estimates_.insert(B(cur_frame_), imu_bias_);

  // Add IMU factor.
  graph_.emplace_shared<gtsam::CombinedImuFactor>(
      X(cur_frame_ - 1), V(cur_frame_ - 1), X(cur_frame_), V(cur_frame_),
      B(cur_frame_ - 1), B(cur_frame_), *odometer_);

  // Reset odometer.
  odometer_->resetIntegration();
}

void GraphManager::SaveGraph(std::ofstream &filename){
    graph_.saveGraph(filename);
}

gtsam::Values GraphManager::OptimizeOnce(){
    return gtsam::LevenbergMarquardtOptimizer(graph_, initial_estimates_).optimize();
}

void GraphManager::Print(){
    graph_.print("-----------FACTOR GRAPH----------");
}

void GraphManager::PrintInitialEstimates(){
    initial_estimates_.print("---------INITIAL ESTIMATES----------");
}

} // namespace dockslam.
