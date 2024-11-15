/*
 * @file usbl_global_factor.h
 * @brief Custom GTSAM factor to constrain the 3d pose
 * of the target using its USBL relative position measurement
 * of the chaser.
 */

#ifndef BACKEND_USBL_GLOBAL_FACTOR_H_
#define BACKEND_USBL_GLOBAL_FACTOR_H_

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include "backend/usbl_measurement.h"

namespace gtsam {

// Prior (unary) factor constraining the target's pose
// by using the USBL position fix from the target's USBL
// to the chaser's USBL, expressed in the target's USBL frame
// of reference. By using the chaser's global navigation
// solution as ground truth, we transform the USBL measurement
// to express the position of the target wrt the world frame.
//   Pose3  : target's 3D pose on SE(3) at time i
class UsblGlobalFactor
    : public NoiseModelFactor1<Pose3> {
 public:
  // Default constructor, only for serialization.
  UsblGlobalFactor() {}

  // Constructor. Here we need the Keys, measurements, and noise model, where
  //   T_i:     key to target's 3D pose on SE(3) at time i
  //   z:       struct with the usbl measurement, chaser pose, and both usbl
  //            extrinsics.
  //   model:   noise model
  UsblGlobalFactor(const Key &T_i,
                   const dockslam::UsblMeasurement &z,
                   const SharedNoiseModel &model);

  // Destructor.
  ~UsblGlobalFactor() override {}

  // Implements the actual measurement model equations and Jacobians, where
  //   T_i:       Target's full 3D pose on SE(3) at time i
  //   H1:        Jacobian of error wrt T_i
  Vector
  evaluateError(const Pose3 &T_i,
                boost::optional<Matrix &> H = boost::none) const override;

  // Print function needed for Testable unit tests.
  void print(const std::string& s = "",
             const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

  // Equals needed for Testable unit tests.
  bool equals(const UsblGlobalFactor& factor, double tol = 1e-9) const;

 private:
  typedef NoiseModelFactor1<Pose3> Base;

  // Usbl measurement and poses required to transform it to the world frame.
  dockslam::UsblMeasurement z_;

};

} // namespace gtsam

#endif  // BACKEND_USBL_GLOBAL_FACTOR_H_
