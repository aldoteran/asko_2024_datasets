#include "noise_models.h"

namespace dockslam {

/*
*Sigma_t = E[delta_C_t * delta_C_t^T]
*        + C_rot * Cx_t^^ * E[delta_C_rot * delta_C_rot^T](C_rot * Cx_t^^)^T
*        + Te_rot(Tx_rot * Tz_t)^^ * E[delta_Ti_rot * delta_Ti_rot^T] * (Te_rot(Tx_rot * Tz_t)^^)^T
*        + Te_rot * Tx_rot * E[delta_z_t * delta_z_t^T] * (Te_rot * Tx_rot)^T
*/
void ComputeUsblGlobalFactorNoise(
    const UsblMeasurement &z,
    const gtsam::Pose3 &init_pose,
    gtsam::noiseModel::Gaussian::shared_ptr &factor_noise,
    const gtsam::noiseModel::Gaussian::shared_ptr &z_noise,
    const gtsam::noiseModel::Gaussian::shared_ptr &chaser_noise,
    const gtsam::noiseModel::Gaussian::shared_ptr &init_noise){
 // Chaser covariances.
 const gtsam::Matrix c_cov = chaser_noise->covariance();
 const gtsam::Matrix c_rot_cov = c_cov.block(0, 0, 3, 3);
 const gtsam::Matrix c_pos_cov = c_cov.block(3, 3, 3, 3); // First term.

 // Target covariances.
 const gtsam::Matrix t_cov = init_noise->covariance();
 const gtsam::Matrix t_rot_cov = t_cov.block(0, 0, 3, 3);
 const gtsam::Matrix t_pos_cov = t_cov.block(3, 3, 3, 3);

 const gtsam::Point3 cx_t = z.c_tfm_cu.translation();
 const gtsam::Matrix cx_t_hat =
     gtsam::skewSymmetric(cx_t.x(), cx_t.y(), cx_t.z());
 const gtsam::Matrix c_rot_tx_hat = z.w_tfm_c.rotation().matrix() * cx_t_hat;
 // C_rot * Cx_t^^ * E[delta_C_rot * delta_C_rot^T](C_rot * Cx_t^^)^T
 const gtsam::Matrix second =
     c_rot_tx_hat * c_rot_cov * gtsam::trans(c_rot_tx_hat); // Second term.

 const gtsam::Point3 tx_rot_tz = z.t_tfm_tu.rotation() * z.t_tu_trans_cu;
 const gtsam::Matrix tx_rot_tz_hat =
     gtsam::skewSymmetric(tx_rot_tz.x(), tx_rot_tz.y(), tx_rot_tz.z());
 // Te_rot(Tx_rot * Tz_t)^^
 const gtsam::Matrix t_rot_tx_rot_tz_hat = init_pose.rotation().matrix() * tx_rot_tz_hat;
 // Te_rot(Tx_rot * Tz_t)^^ * E[delta_Ti_rot * delta_Ti_rot^T] * (Te_rot(Tx_rot * Tz_t)^^)^T
 const gtsam::Matrix third =
     t_rot_tx_rot_tz_hat * t_rot_cov * gtsam::trans(t_rot_tx_rot_tz_hat); // Third term.

 const gtsam::Rot3 t_rot_tx_rot =
     init_pose.rotation() * z.t_tfm_tu.rotation();
 // Te_rot * Tx_rot * E[delta_z_t * delta_z_t^T] * (Te_rot * Tx_rot)^T
 const gtsam::Matrix fourth = t_rot_tx_rot.matrix() * z_noise->covariance() *
                              gtsam::trans(t_rot_tx_rot.matrix()); // Fourth term.

 factor_noise = gtsam::noiseModel::Gaussian::Covariance(c_pos_cov + second + third + fourth);
}


void ComputeOpticalGlobalFactorNoise(
    const gtsam::Pose3 &z,
    gtsam::noiseModel::Gaussian::shared_ptr &factor_noise,
    const gtsam::Pose3 &chaser_camera_extrinsics,
    const gtsam::Pose3 &target_fiducials_extrinsics,
    const gtsam::noiseModel::Gaussian::shared_ptr &z_noise,
    const gtsam::noiseModel::Gaussian::shared_ptr &chaser_noise){

 // -------- rotation covariance ------
 // Chaser covariances.
 const gtsam::Matrix c_cov = chaser_noise->covariance();
 const gtsam::Matrix c_rot_cov = c_cov.block(0, 0, 3, 3);
 // Measurement covariances.
 const gtsam::Matrix z_cov = chaser_noise->covariance();
 const gtsam::Matrix z_rot_cov = c_cov.block(0, 0, 3, 3);

 // Rotations.
 const gtsam::Rot3 c_rot_cam = chaser_camera_extrinsics.rotation();
 const gtsam::Rot3 t_rot_fid = target_fiducials_extrinsics.rotation();
 const gtsam::Rot3 cam_rot_fid = z.rotation();

 // ( c_rot_cam * cam_rot_fid * fid_rot_t )
 const gtsam::Matrix rot_chain = c_rot_cam.matrix() * cam_rot_fid.matrix() * t_rot_fid.transpose();
 // (C_rot_cam * cam_rot_fid * fid_rot_T)^T * E[delta_C_rot * delta_C_rot^T]
 // * (C_rot_cam * cam_rot_fid * fid_rot_T)
 const gtsam::Matrix first = rot_chain.transpose() * c_rot_cov * rot_chain;

 // fid_rot_T^T * E[delta_rot_z * delta_rot_z^T] * fid_rot_T
 const gtsam::Matrix second = t_rot_fid.matrix() * z_rot_cov * t_rot_fid.transpose();

 std::cout << "Optical rot covariance:\n" << first + second << std::endl;

 // --------- translation covariance ----------
 gtsam::Matrix factor_cov = gtsam::Matrix6::Identity();
 factor_cov.block(0,0,3,3) *= first + second;

 factor_noise = gtsam::noiseModel::Gaussian::Covariance(factor_cov);
}

}  // namespace dockslam
