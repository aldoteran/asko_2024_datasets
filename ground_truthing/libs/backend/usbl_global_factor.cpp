#include "usbl_global_factor.h"

namespace gtsam {

UsblGlobalFactor::UsblGlobalFactor(
    const Key &T_i, const dockslam::UsblMeasurement &z,
    const SharedNoiseModel &model)
    : Base(model, T_i), z_(z) {}

Vector UsblGlobalFactor::evaluateError(
    const Pose3 &T_i, boost::optional<Matrix &> H) const {

    // Target's pose in the world frame (estimated).
    const Rot3 w_rot_t = T_i.rotation();
    const Point3 w_trans_t = T_i.translation();
    // Target->Target's usbl extrinsic calibration.
    const Rot3 t_rot_tu = z_.t_tfm_tu.rotation();
    const Point3 t_trans_tu = z_.t_tfm_tu.translation();

    // Chaser's pose in the world frame (measured).
    const Rot3 w_rot_c =  z_.w_tfm_c.rotation();
    const Point3 w_trans_c = z_.w_tfm_c.translation();
    // Chaser->Chaser's usbl extrinsic calibration.
    const Point3 c_trans_cu = z_.c_tfm_cu.translation();

    /*
     * The error is defined as:
     * e = W_t_C_W + W_R_C * C_t_Cu_C
     *     - W_R_T (T_R_Tu * Tu_t_Tu_Cu + T_t_T_tu) ---> get Jacobian
     *     - W_t_T_W
     */
    Matrix3 J_err_theta;
    Matrix3 J_err_v; // Not used.
    Vector3 t_rot_err = - w_rot_t.rotate(
        t_rot_tu * z_.t_tu_trans_cu + t_trans_tu, J_err_v, J_err_theta);
    Vector3 error = w_trans_c + w_rot_c * c_trans_cu + t_rot_err - w_trans_t;
    //Vector3 error = w_trans_c + w_rot_c * c_trans_cu -
                    //w_rot_t * t_rot_tu * z_.t_tu_trans_cu -
                    //w_rot_t * t_trans_tu - w_trans_c;

    // Compute the Jacobians (if requested).
    if (H) {
      /*
       * The Jacobian of the error above wrt the target's pose in se(3),
       * i.e. the R6 representation of the pose. Zeroing out the constant terms,
       * we're looking for the derivative:
       *
       * d(error)/d(xi) = [ d(error)/d(theta) d(error)/d(t) ]_3x6
       *
       * where:
       *
       * d(error)/d(theta) = -(T_R_Tu*Tu_t_Tu_Cu+T_t_T_tu)*d(Exp(theta))/d(theta)
       *                   = - (c) * d(Exp(theta))/d(theta)
       *                   = - W_R_T * [c]_x * J_r(theta)
       * with [.]_x being the skew-symmetric matrix of (c) and J_r(theta) is the
       * right Jacobian of the ExpMap of SO(3), and
       *
       * d(error)/d(t) = I
       */
      H->resize(3, 6);
      H->block<3, 3>(0, 0) << J_err_theta;
      H->block<3, 3>(0, 3) << Matrix3::Identity();
    }

      // Return the full error.
      return error;
}

void UsblGlobalFactor::print(const std::string &s,
                             const KeyFormatter &keyFormatter) const {

  std::cout << s << std::endl;
  std::cout << "USBL global factor on " << keyFormatter(this->key())
            << std::endl;
  std::cout << "\t USBL measurement mean:" << std::endl;
  std::cout << z_.t_tu_trans_cu << std::endl;
  this->noiseModel_->print("  noise model: ");
  //std::cout << "\t Noise model (covariance):" << std::endl;
  //std::cout << this->noiseModel_->covariance() << std::endl;
  return this->printKeys(s, keyFormatter);
}

bool UsblGlobalFactor::equals(const UsblGlobalFactor &factor, double tol)
    const {
  return Base::equals(factor);
}

}  // namespace gtsam
