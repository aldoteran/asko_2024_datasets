#include "depth_factor.h"

namespace gtsam {

DepthFactor::DepthFactor(const Key& j, double z, const SharedNoiseModel& model)
    : Base(model, j), mz_(z) {}

Vector DepthFactor::evaluateError(const Pose3& q,
                                  boost::optional<Matrix&> H) const {

    // Get the rotation in Euler angles for Jacobian.
  const Vector3 ypr = q.rotation().ypr();
  const double sin_p = std::sin(ypr(1));
  const double sin_r = std::sin(ypr(2));
  const double cos_p = std::cos(ypr(1));
  const double cos_r = std::cos(ypr(2));

  // Compute the error components.
  double error_z = q.z() - mz_;

  // Compute the Jacobians (if requested).
  if (H) {  // 1x6, wrt q.
    *H = (Matrix(1, 6) << 0.0, 0.0, 0.0, -sin_p, cos_p * sin_r, cos_p * cos_r).finished();
  }

  // Assemble and return the full error.
  return (Vector(1) << error_z).finished();
}

void DepthFactor::print(const std::string& s,
                              const KeyFormatter& keyFormatter) const {
  // TODO(tonioteran) Implement custom print?
  std::cout << "Depth Factor\n";
}

bool DepthFactor::equals(const DepthFactor& factor,
                               double tol) const {
  return Base::equals(factor);
}

}  // namespace gtsam
