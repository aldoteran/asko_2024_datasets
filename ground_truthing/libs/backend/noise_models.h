/*
 * @file noise_models.h
 * @brief Noise models for the global USBL and optical relative
 *        measurement factor.
 */

#ifndef BACKEND_NOISE_MODELS_H_
#define BACKEND_NOISE_MODELS_H_

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include "backend/usbl_measurement.h"

namespace dockslam {

/*
 * Computes the covariance of the residual (error) for the UsblGlobalFactor:
 *          e = W_t_T - h_t
 * where W_t_T is the target's usbl fix transformed using the chaser's ground
 * truth pose to be expressed in the World frame. The covariance is computed as:
 *  Sigma_t = E[delta_C_t * delta_C_t^T]
 *          + C_rot * Cx_t^^ * E[delta_C_rot * delta_C_rot^T](C_rot * Cx_t^^)^T
 *          + Te_rot(Tx_rot * Tz_t)^^ * E[delta_Ti_rot * delta_Ti_rot^T] * (Te_rot(Tx_rot * Tz_t)^^)^T
 *          + Te_rot * Tx_rot * E[delta_z_t * delta_z_t^T] * (Te_rot * Tx_rot)^T
 *  where:
 *    E[delta_C_t * delta_C_t^T] = the chaser's position covariance in the world frame (given by INS).
 *    E[delta_C_rot * delta_C_rot^T] = chaser's rotation covariance in the world frame (given by INS).
 *    E[delta_Ti_rot * delta_Ti_rot^T] = Target's position initial covariance (initial conditions).
 *    E[delta_z_t * delta_z_t^T] = tagret's usbl measurement noise.
 *    C_rot = chaser's rotation wrt the world frame (measured).
 *    Cx_t = chaser's frame to usbl's extrinsic calibration (translation).
 *    Te_rot = target's estimated rotation wrt the world frame.
 *    Tx_rot = target's frame to usbl's extrinsic calibration (rotation).
 *    Tz_t = raw usbl measurement in the target's frame.
 * and:
 *    ^^: hat operator.
 *    ^T: matrix transpose operator.
 */
void ComputeUsblGlobalFactorNoise(
    const UsblMeasurement &z,
    const gtsam::Pose3 &init_pose,
    gtsam::noiseModel::Gaussian::shared_ptr &factor_noise,
    const gtsam::noiseModel::Gaussian::shared_ptr &z_noise,
    const gtsam::noiseModel::Gaussian::shared_ptr &chaser_noise,
    const gtsam::noiseModel::Gaussian::shared_ptr &init_noise);

/*
 * Computes the covariance of the residual for the optical measurement's prior global factor.
 *          e = (W_tfm_C * C_tfm_B)^-1 * W_tfm_T
 * We split the covariance terms into rotation, translation, and cross-terms.
 *  Sigma_rot = (C_rot_cam * cam_rot_fid * fid_rot_T)^T * E[delta_C_rot * delta_C_rot^T]
 *            * (C_rot_cam * cam_rot_fid * fid_rot_T)
 *            + fid_rot_T^T * E[delta_rot_z * delta_rot_z^T] * fid_rot_T
 * where:
 *   E[delta_C_rot * delta_C_rot^T] = chaser's rotation covariance.
 *   E[delta_rot_z * delta_rot_z^T] = camera pose estimate's rotation covariance.
 *   C_rot_cam = chaser to camera extrinsic rotation.
 *   cam_rot_fid = camera to fiducial (measurement) rotation.
 *   fid_rot_T = fiducials to target extrinsics rotation.
 */
void ComputeOpticalGlobalFactorNoise(
    const gtsam::Pose3 &z, const gtsam::Pose3 &init_pose,
    const gtsam::Pose3 &chaser_pose,
    gtsam::noiseModel::Gaussian::shared_ptr &factor_noise,
    const gtsam::Pose3 &chaser_camera_extrinsics,
    const gtsam::Pose3 &target_fiducials_extrinsics,
    const gtsam::noiseModel::Gaussian::shared_ptr &z_noise,
    const gtsam::noiseModel::Gaussian::shared_ptr &chaser_noise);

// Uses the GTSAM Adjoints machinery to compute the noise distribution
// for the relative pose T_ab = T_a^-1 T_b.
// We follow the derivations in https://gtsam.org/2021/02/23/uncertainties-part3.html.
void ComputeRelativeTfmNoise(
    const gtsam::Pose3 &tfm_a, const gtsam::Pose3 &tfm_b,
    const gtsam::noiseModel::Gaussian::shared_ptr &tfm_a_noise,
    const gtsam::noiseModel::Gaussian::shared_ptr &tfm_b_noise,
    gtsam::noiseModel::Gaussian::shared_ptr &factor_noise);

} // namespace dockslam

#endif  // BACKEND_NOISE_MODELS_H_
