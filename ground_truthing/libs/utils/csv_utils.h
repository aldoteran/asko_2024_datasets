#ifndef _UTILS_CSV_UTILS_H_
#define _UTILS_CSV_UTILS_H_

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Quaternion.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#define OPTICAL 1
#define USBL 2

using gtsam::symbol_shorthand::X; // Target's pose.

namespace csv_utils{

void AppendOpticalKeyframe(std::vector<std::string> &csvdata,
                           const gtsam::Pose3 &meas, const gtsam::Pose3 &chaser,
                           const gtsam::Pose3 &target, double stamp);

void AppendUsblKeyframe(std::vector<std::string> &csvdata,
                        const gtsam::Point3 &meas, const gtsam::Pose3 &chaser,
                        const gtsam::Pose3 &target, double stamp);

void DataToCsvFile(const std::vector<std::string> &data,
                   const std::string &filename);

void ValuesToCsvFile(const gtsam::Values &values,
                     const std::map<int, double> &timestamps,
                     const std::string &path_to_data,
                     const std::vector<int> &optical_frames = {},
                     const std::vector<int> &usbl_frames = {});

void SplitKeyframes(const std::vector<std::string> &data,
                    const std::vector<int> &frames, const std::string &filename);

} // namespace csv_utils

#endif // _UTILS_CSV_UTILS_H_
