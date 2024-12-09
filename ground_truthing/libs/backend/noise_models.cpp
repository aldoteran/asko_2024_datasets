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

 factor_noise = gtsam::noiseModel::Gaussian::Covariance(c_pos_cov + second +
                                                        third + fourth);
}

void ComputeOpticalGlobalFactorNoise(
    const gtsam::Pose3 &z, const gtsam::Pose3 &init_pose,
    const gtsam::Pose3 &chaser_pose,
    gtsam::noiseModel::Gaussian::shared_ptr &factor_noise,
    const gtsam::Pose3 &chaser_camera_extrinsics,
    const gtsam::Pose3 &target_fiducials_extrinsics,
    const gtsam::noiseModel::Gaussian::shared_ptr &z_noise,
    const gtsam::noiseModel::Gaussian::shared_ptr &chaser_noise) {

  // Chaser covariances.
  const gtsam::Matrix c_cov = chaser_noise->covariance();
  const gtsam::Matrix c_rot_cov = c_cov.block(0, 0, 3, 3);
  const gtsam::Matrix c_t_cov = c_cov.block(3, 3, 3, 3);
  const gtsam::Matrix c_cross_cov = c_cov.block(0, 3, 3, 3); // Upper-right block.
  // Measurement covariances.
  const gtsam::Matrix z_cov = z_noise->covariance();
  const gtsam::Matrix z_rot_cov = z_cov.block(0, 0, 3, 3);
  const gtsam::Matrix z_t_cov = z_cov.block(3, 3, 3, 3);
  const gtsam::Matrix z_cross_cov = z_cov.block(0, 3, 3, 3); // Upper-right block.
  // Rotations.
  const gtsam::Matrix /*w_rot_t*/ R = init_pose.rotation().matrix(); // R.
  const gtsam::Matrix /*t_rot_w*/ R_T = R.transpose(); // R_T.
  const gtsam::Matrix /*w_rot_c*/ R1 = chaser_pose.rotation().matrix(); // R1.
  const gtsam::Matrix /*c_rot_cam*/ R2 =
      chaser_camera_extrinsics.rotation().matrix();               // R2.
  const gtsam::Matrix /*cam_rot_fid*/ R3 = z.rotation().matrix(); // R3
  const gtsam::Matrix /*fid_rot_t*/ R4 =
      target_fiducials_extrinsics.rotation().transpose(); // R4.
  const gtsam::Matrix /*t_rot_fid*/ R4_T = R4.transpose();
  // Translations.
  const gtsam::Point3 /*c_trans_cam*/ t2 =
      chaser_camera_extrinsics.translation();
  const gtsam::Point3 /*cam_trans_fid*/ t3 = z.translation();
  const gtsam::Point3 /*fid_fid_trans_t*/ t4 =
      target_fiducials_extrinsics.inverse().translation();
  // Skew symmetric translation matrices.
  const gtsam::Matrix /*c_c_trans_cam_hat*/ t2_hat =
      gtsam::skewSymmetric(t2.x(), t2.y(), t2.z());
  const gtsam::Matrix /*fid_fid_trans_t_hat*/ t4_hat =
      gtsam::skewSymmetric(t4.x(), t4.y(), t4.z());

  // -------- rotation covariance calculation ------

  // ( c_rot_cam * cam_rot_fid * fid_rot_t )
  const gtsam::Matrix rot_chain = R2 * R3 * R4;
  // (C_rot_cam * cam_rot_fid * fid_rot_T)^T * E[delta_C_rot * delta_C_rot^T]
  // * (C_rot_cam * cam_rot_fid * fid_rot_T)
  const gtsam::Matrix first = rot_chain.transpose() * c_rot_cov * rot_chain;
  // fid_rot_T^T * E[delta_rot_z * delta_rot_z^T] * fid_rot_T
  const gtsam::Matrix second = R4_T * z_rot_cov * R4;

  const gtsam::Matrix rot_cov = first + second;

  // --------- translation covariance calculation ----------

  const gtsam::Vector3 R2R3t4 = R2 * R3 * t4;
  const gtsam::Vector3 R2t3 = R2 * t3;
  const gtsam::Matrix R2R3t4_hat =
      gtsam::skewSymmetric(R2R3t4(0), R2R3t4(1), R2R3t4(2));
  const gtsam::Matrix R2t3_hat =
      gtsam::skewSymmetric(R2t3(0), R2t3(1), R2t3(2));

  // Following optical_factor.md, we compute matrices A, B, C, and D.
  const gtsam::Matrix A = R_T * R1 * R2 * R3 * t4_hat;
  const gtsam::Matrix A_T = A.transpose();
  const gtsam::Matrix B = R_T * R1 * (R2R3t4_hat + R2t3_hat + t2_hat);
  const gtsam::Matrix B_T = B.transpose();
  const gtsam::Matrix C = R_T * R1 * R2 * R3;
  const gtsam::Matrix C_T = C.transpose();
  const gtsam::Matrix D = R_T * R1;
  const gtsam::Matrix D_T = D.transpose();

  const gtsam::Matrix trans_cov =
      A * z_rot_cov * A_T + A * z_cross_cov * C_T + C * z_t_cov * C_T +
      C * z_cross_cov.transpose() * A_T + B * c_rot_cov * B_T -
      B * c_cross_cov * D_T + D * c_t_cov * D_T -
      D * c_cross_cov.transpose() * B_T;

  // --------- cross-covariance calculation ----------

  // Following optical_factor.md, we compute matrices alpha.
  const gtsam::Matrix alpha = (R2 * R3 * R4).transpose();
  const gtsam::Matrix cross_cov = -alpha * c_rot_cov * B_T +
                                  alpha * c_cross_cov * D_T -
                                  R4_T * z_rot_cov * A_T +
                                  R4_T * z_cross_cov * C_T;

  std::cout << "Optical rot covariance:\n" << rot_cov << std::endl;
  std::cout << "Optical trans covariance:\n" << trans_cov << std::endl;
  std::cout << "Optical cross covariance:\n" << cross_cov << std::endl;

  gtsam::Matrix6 factor_cov;
  factor_cov.block(0, 0, 3, 3) = rot_cov ;
  factor_cov.block(3, 3, 3, 3) = trans_cov;
  factor_cov.block(0, 3, 3, 3) = cross_cov;
  factor_cov.block(3, 0, 3, 3) = cross_cov.transpose();

  std::cout << "Optical factor cov: \n" << factor_cov.matrix() << "\n";

  // TODO: We're missing the last piece of the puzzle: the inverse composition
  // cov.

  factor_noise = gtsam::noiseModel::Gaussian::Covariance(factor_cov);
}

}  // namespace dockslam
