/**
 * @file depth_factor.h
 * @brief Custom GTSAM factor for constraining the depth (Z) of a 6DOF pose.
 */

#ifndef GTSAM_UTILS_DEPTH_FACTOR_H_
#define GTSAM_UTILS_DEPTH_FACTOR_H_

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>

namespace gtsam {

// Prior factor for a 6-DOF pose variable node that contrains the Z
// (depth on an AUV) degree with a single measurement.
// The template variable correspond as follows:
//   Pose3 : AUV's 6-DOF variable node
class DepthFactor : public NoiseModelFactor1<Pose3> {
public:
  // Default constructor, only for serialization.
  DepthFactor() {}

  // Constructor. Here we need the Key, measurement, and noise model, where
  //   j:     key to AUV's 3-DOF pose that is to be constrained
  //   z:     measured depth
  //   model:   noise model
  DepthFactor(const Key& j, double z, const SharedNoiseModel& model);
      //NoiseModelFactor1<Pose3>(model, j), mz_(z) {}

  // Destructor.
  ~DepthFactor() override {}

  // Implements the actual measurement model equations and Jacobians, where
  //   q: AUV's 3D pose
  //   H: Jacobian of the observation model wrt q
  Vector evaluateError(
      const Pose3& q, boost::optional<Matrix&> H = boost::none) const override;

  // Print function needed for Testable unit tests.
  void print(const std::string& s = "",
             const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

  // Equals needed for Testable unit tests.
  bool equals(const DepthFactor& factor, double tol = 1e-9) const;

 private:
  typedef NoiseModelFactor1<Pose3> Base;

  // Variable to store the depth measurement [m].
  double mz_;
};

} // namespace gtsam

#endif  // GTSAM_UTILS_DEPTH_FACTOR_H_
