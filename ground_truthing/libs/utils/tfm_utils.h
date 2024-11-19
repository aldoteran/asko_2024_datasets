
#ifndef _UTILS_TFM_UTILS_H_

#define _UTILS_TFM_UTILS_H_

#include <Eigen/Core>
#include <gtsam/geometry/Rot3.h>

namespace dockslam{

// Rotates a heading angle expressed in the NED global frame to ENU.
double NedHeadingInEnu(double ned_angle, bool degrees);

} // namespace dockslam.

#endif // _UTILS_TFM_UTILS_H_
