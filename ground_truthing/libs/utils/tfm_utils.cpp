
#include "tfm_utils.h"

namespace dockslam{

double NedHeadingInEnu(double ned_angle, bool in_degrees) {
  if (in_degrees) {
   ned_angle = ned_angle * M_PI / 180.0;
  }
  // NED_rot_z.
  Eigen::Matrix3d ned_rot;
  ned_rot << std::cos(ned_angle), -std::sin(ned_angle), 0.0,
             std::sin(ned_angle), std::cos(ned_angle), 0.0,
             0.0, 0.0, 1.0;

  // To convert from NED to ENU, we must roll 180 and yaw -90 degs.
  Eigen::Matrix3d roll_180;
  roll_180 << 1.0, 0.0, 0.0,
              0.0, std::cos(M_PI), -std::sin(M_PI),
              0.0, std::sin(M_PI), std::cos(M_PI);
  Eigen::Matrix3d yaw_90;
  yaw_90 << std::cos(M_PI/2.0), -std::sin(M_PI/2.0), 0.0,
                std::sin(M_PI/2.0), std::cos(M_PI/2.0), 0.0,
                0.0, 0.0, 1.0;

  // ENU_rot_NED.
  const Eigen::Matrix3d enu_to_ned = yaw_90 * roll_180;
  // ENU_rot_z = ENU_rot_NED * NED_rot_z.
  const gtsam::Rot3 enu_rot = gtsam::Rot3(enu_to_ned * ned_rot);
  if (in_degrees){
      return enu_rot.yaw() * 180.0 / M_PI;
  }

  return enu_rot.yaw();
}

} // namespace dockslam.
