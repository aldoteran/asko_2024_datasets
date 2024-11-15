/*
 * @file usbl_measurement.h
 * @brief Definition of the usbl measurement struct used
 *        as input for the UsblGlobalFactor.
 * @date Nov 11, 2024
 * @author aldo terán (aldot@kth.se)
 * @author tonio terán (teran@mit.edu)
 */


#ifndef _BACKEND_USBL_MEASUREMENT_H_
#define _BACKEND_USBL_MEASUREMENT_H_

#include <gtsam/geometry/Pose3.h>

namespace dockslam{

// Used as input to the UsblGlobalFactor.
struct UsblMeasurement {
  gtsam::Point3 t_tu_trans_cu; // Raw measurement. T_Tusbl_trans_Cusbl.
  gtsam::Pose3 w_tfm_c;        // W_tfm_C.
  gtsam::Pose3 c_tfm_cu;       // C_tfm_Cusbl.
  gtsam::Pose3 t_tfm_tu;       // T_tfm_Tusbl.
  };

} // namespace dockslam.

#endif // _BACKEND_USBL_MEASUREMENT_H_
