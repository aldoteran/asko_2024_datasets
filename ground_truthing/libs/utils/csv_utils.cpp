
#include "csv_utils.h"

namespace csv_utils{

void AppendOpticalKeyframe(std::vector<std::string> &csvdata,
                           const gtsam::Pose3 &meas, const gtsam::Pose3 &chaser,
                           const gtsam::Pose3 &target, double stamp) {
  // We have to make a really long string that will be the entire row of the
  // csv file.
  std::string row = std::to_string(stamp) + "," + std::to_string(OPTICAL) + ",";

  // Poses are going to be added as quat + xyz.
  const gtsam::Quaternion q_meas = meas.rotation().toQuaternion();
  const gtsam::Quaternion q_chaser = chaser.rotation().toQuaternion();
  const gtsam::Quaternion q_target = target.rotation().toQuaternion();

  // Add measured optical pose.
  row += std::to_string(q_meas.x()) + "," + std::to_string(q_meas.y()) + "," +
         std::to_string(q_meas.z()) + "," + std::to_string(q_meas.w()) + "," +
         std::to_string(meas.x()) + "," + std::to_string(meas.y()) + "," +
         std::to_string(meas.z()) + ",";
  // Add chaser pose.
  row += std::to_string(q_chaser.x()) + "," + std::to_string(q_chaser.y()) +
         "," + std::to_string(q_chaser.z()) + "," +
         std::to_string(q_chaser.w()) + "," + std::to_string(chaser.x()) + "," +
         std::to_string(chaser.y()) + "," + std::to_string(chaser.z()) + ",";
  // Add target pose.
  row += std::to_string(q_target.x()) + "," + std::to_string(q_target.y()) +
         "," + std::to_string(q_target.z()) + "," +
         std::to_string(q_target.w()) + "," + std::to_string(target.x()) + "," +
         std::to_string(target.y()) + "," + std::to_string(target.z());

  csvdata.push_back(row);
}

void AppendUsblKeyframe(std::vector<std::string> &csvdata,
                        const gtsam::Point3 &meas, const gtsam::Pose3 &chaser,
                        const gtsam::Pose3 &target, double stamp) {
  std::string row = std::to_string(stamp) + "," + std::to_string(USBL) + ",";

  // Poses are going to be added as quat + xyz.
  const gtsam::Quaternion q_chaser = chaser.rotation().toQuaternion();
  const gtsam::Quaternion q_target = target.rotation().toQuaternion();

  // Add measured usbl position. Add dummy orientation to the measurement.
  row += "0,0,0,0," + std::to_string(meas.x()) + "," +
         std::to_string(meas.y()) + "," + std::to_string(meas.z()) + ",";
  // Add chaser pose.
  row += std::to_string(q_chaser.x()) + "," + std::to_string(q_chaser.y()) +
         "," + std::to_string(q_chaser.z()) + "," +
         std::to_string(q_chaser.w()) + "," + std::to_string(chaser.x()) + "," +
         std::to_string(chaser.y()) + "," + std::to_string(chaser.z()) + ",";
  // Add target pose.
  row += std::to_string(q_target.x()) + "," + std::to_string(q_target.y()) +
         "," + std::to_string(q_target.z()) + "," +
         std::to_string(q_target.w()) + "," + std::to_string(target.x()) + "," +
         std::to_string(target.y()) + "," + std::to_string(target.z());

  csvdata.push_back(row);
}

void DataToCsvFile(const std::vector<std::string> &data,
                   const std::string &filename) {

  std::ofstream csvfile(filename);
  for (const std::string row : data){
    csvfile << row << std::endl;
  }
  csvfile.close();
}

} // namespace csv_utils
