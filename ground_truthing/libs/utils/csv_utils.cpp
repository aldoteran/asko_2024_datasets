
#include "csv_utils.h"

namespace csv_utils {

void AppendOpticalKeyframe(std::vector<std::string> &csvdata,
                           const gtsam::Pose3 &meas, const gtsam::Pose3 &chaser,
                           const gtsam::Pose3 &target, double stamp, bool as_quat) {
  // We have to make a really long string that will be the entire row of the
  // csv file.
  std::string row = std::to_string(stamp) + "," + std::to_string(OPTICAL) + ",";

  if (as_quat) {
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
           std::to_string(q_chaser.w()) + "," + std::to_string(chaser.x()) +
           "," + std::to_string(chaser.y()) + "," + std::to_string(chaser.z()) +
           ",";
    // Add target pose.
    row += std::to_string(q_target.x()) + "," + std::to_string(q_target.y()) +
           "," + std::to_string(q_target.z()) + "," +
           std::to_string(q_target.w()) + "," + std::to_string(target.x()) +
           "," + std::to_string(target.y()) + "," + std::to_string(target.z());
  } else {
    // Add meas pose.
    const gtsam::Matrix m_m = meas.rotation().matrix();
    row += std::to_string(m_m(0, 0)) + "," + std::to_string(m_m(0, 1)) + "," +
           std::to_string(m_m(0, 2)) + "," + std::to_string(m_m(1, 1)) + "," +
           std::to_string(m_m(1, 2)) + "," + std::to_string(m_m(2, 2)) + "," +
           std::to_string(meas.x()) + "," + std::to_string(meas.y()) + "," +
           std::to_string(meas.z()) + ",";
    // Add chaser pose.
    const gtsam::Matrix c_m = chaser.rotation().matrix();
    row += std::to_string(c_m(0, 0)) + "," + std::to_string(c_m(0, 1)) + "," +
           std::to_string(c_m(0, 2)) + "," + std::to_string(c_m(1, 0)) + "," +
           std::to_string(c_m(1, 1)) + "," + std::to_string(c_m(1, 2)) + "," +
           std::to_string(c_m(2, 0)) + "," + std::to_string(c_m(2, 1)) + "," +
           std::to_string(c_m(2, 2)) + "," + std::to_string(chaser.x()) + "," +
           std::to_string(chaser.y()) + "," + std::to_string(chaser.z()) + ",";
    // Add target pose.
    const gtsam::Matrix t_m = target.rotation().matrix();
    row += std::to_string(t_m(0, 0)) + "," + std::to_string(t_m(0, 1)) + "," +
           std::to_string(t_m(0, 2)) + "," + std::to_string(t_m(1, 0)) + "," +
           std::to_string(t_m(1, 1)) + "," + std::to_string(t_m(1, 2)) + "," +
           std::to_string(t_m(2, 0)) + "," + std::to_string(t_m(2, 1)) + "," +
           std::to_string(t_m(2, 2)) + "," + std::to_string(target.x()) + "," +
           std::to_string(target.y()) + "," + std::to_string(target.z()) + ",";
  }


  csvdata.push_back(row);
}

void AppendUsblKeyframe(std::vector<std::string> &csvdata,
                        const gtsam::Point3 &meas, const gtsam::Pose3 &chaser,
                        const gtsam::Pose3 &target, double stamp, bool as_quat) {
  std::string row = std::to_string(stamp) + "," + std::to_string(USBL) + ",";

  if (as_quat){
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
  } else {
    // Add measured usbl position. Add dummy orientation to the measurement.
    row += "0,0,0,0,0,0,0,0,0," + std::to_string(meas.x()) + "," +
           std::to_string(meas.y()) + "," + std::to_string(meas.z()) + ",";
    // Add chaser pose.
    const gtsam::Matrix c_m = chaser.rotation().matrix();
    row += std::to_string(c_m(0, 0)) + "," + std::to_string(c_m(0, 1)) + "," +
           std::to_string(c_m(0, 2)) + "," + std::to_string(c_m(1, 0)) + "," +
           std::to_string(c_m(1, 1)) + "," + std::to_string(c_m(1, 2)) + "," +
           std::to_string(c_m(2, 0)) + "," + std::to_string(c_m(2, 1)) + "," +
           std::to_string(c_m(2, 2)) + "," + std::to_string(chaser.x()) + "," +
           std::to_string(chaser.y()) + "," + std::to_string(chaser.z()) + ",";
    // Add target pose.
    const gtsam::Matrix t_m = target.rotation().matrix();
    row += std::to_string(t_m(0, 0)) + "," + std::to_string(t_m(0, 1)) + "," +
           std::to_string(t_m(0, 2)) + "," + std::to_string(t_m(1, 0)) + "," +
           std::to_string(t_m(1, 1)) + "," + std::to_string(t_m(1, 2)) + "," +
           std::to_string(t_m(2, 0)) + "," + std::to_string(t_m(2, 1)) + "," +
           std::to_string(t_m(2, 2)) + "," + std::to_string(target.x()) + "," +
           std::to_string(target.y()) + "," + std::to_string(target.z()) + ",";
  }

  csvdata.push_back(row);
}

void DataToCsvFile(const std::vector<std::string> &data,
                   const std::string &filename) {

  std::ofstream csvfile(filename);
  for (const std::string row : data) {
    csvfile << row << std::endl;
  }
  csvfile.close();
}

void ValuesToCsvFile(const gtsam::Values &values,
                     const std::map<int, double> &timestamps,
                     const std::string &path_to_data, bool optimized_chaser,
                     const std::vector<int> &optical_frames,
                     const std::vector<int> &usbl_frames) {
  size_t size;
  if (optimized_chaser) {
    size = values.size() / 4;
  } else {
    size = values.size() / 3;
  }

  std::vector<std::string> data;
  data.reserve(size + 1);
  for (size_t i = 0; i < size; i++) {
    std::string row;
    gtsam::Pose3 p = values.at<gtsam::Pose3>(X(i));
    // TODO: Make this into if in_quat.
    if (false) {
      const gtsam::Quaternion q = p.rotation().toQuaternion();
      const gtsam::Point3 t = p.translation();
      double s = timestamps.at(i);
      data.push_back(std::to_string(s) + "," + std::to_string(q.x()) + "," +
                     std::to_string(q.y()) + "," + std::to_string(q.z()) + "," +
                     std::to_string(q.w()) + "," + std::to_string(t.x()) + "," +
                     std::to_string(t.y()) + "," + std::to_string(t.z()));
    }
    gtsam::Point3 t = p.translation();
    gtsam::Matrix m = p.rotation().matrix();
    double s = timestamps.at(i);
    row = std::to_string(s) + "," + std::to_string(m(0, 0)) + "," +
          std::to_string(m(0, 1)) + "," + std::to_string(m(0, 2)) + "," +
          std::to_string(m(1, 0)) + "," + std::to_string(m(1, 1)) + "," +
          std::to_string(m(1, 2)) + "," + std::to_string(m(2, 0)) + "," +
          std::to_string(m(2, 1)) + "," + std::to_string(m(2, 2)) + "," +
          std::to_string(t.x()) + "," + std::to_string(t.y()) + "," +
          std::to_string(t.z());

    if (optimized_chaser) {
      gtsam::Pose3 p = values.at<gtsam::Pose3>(C(i));
      gtsam::Point3 t = p.translation();
      gtsam::Matrix m = p.rotation().matrix();
      row += "," + std::to_string(m(0, 0)) + "," + std::to_string(m(0, 1)) +
             "," + std::to_string(m(0, 2)) + "," + std::to_string(m(1, 0)) +
             "," + std::to_string(m(1, 1)) + "," + std::to_string(m(1, 2)) +
             "," + std::to_string(m(2, 0)) + "," + std::to_string(m(2, 1)) +
             "," + std::to_string(m(2, 2)) + "," + std::to_string(t.x()) + "," +
             std::to_string(t.y()) + "," + std::to_string(t.z());
    }
    data.push_back(row);
  }
  std::string filename = path_to_data + "full_graph_results";
  std::cout << "Saving full graph results to " << filename << std::endl;
  DataToCsvFile(data, filename);

  if (!optical_frames.empty()) {
    filename = path_to_data + "optical_keyframe_results";
    std::cout << "Saving optical keyframe results to " << filename << std::endl;
    SplitKeyframes(data, optical_frames, filename);
  }
  if (!usbl_frames.empty()) {
    filename = path_to_data + "usbl_keyframe_results";
    std::cout << "Saving usbl keyframe results to " << filename << std::endl;
    SplitKeyframes(data, usbl_frames, filename);
  }
}

void SplitKeyframes(const std::vector<std::string> &data,
        const std::vector<int> &frames, const std::string &filename) {
    size_t size = frames.size();
    std::vector<std::string> frame_data;
    frame_data.reserve(size);
    for (int i : frames){
        frame_data.push_back(data[i]);
    }
    DataToCsvFile(frame_data, filename);
}

} // namespace csv_utils
