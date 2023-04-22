/**
 * @file DataUtils.cpp
 * @brief Utility functions for interacting with the raw datasets.
 * @date September 11, 2019
 * @author MIT Space Systems Laboratory
 * @author Antonio Teran (teran@mit.edu)
 * Copyright 2020 MIT Space Systems Laboratory
 */

#include "sph-vertigo/DataUtils.h"

#include <fstream>

namespace sph {

/* ************************************************************************** */
std::vector<MsgImuData> LoadImuData(const std::string &imu_file) {
  // Vector with the IMU measurements.
  std::vector<MsgImuData> measurements;

  // Variables to store text lines and words.
  std::string line, token;

  // Analyze imu file only if it exists.
  std::ifstream file(imu_file.c_str());
  if (file.good()) {
    while (std::getline(file, line)) {
      // Parse the file, line by line.
      std::stringstream strm(line);

      // Get all line values together.
      std::vector<std::string> values;
      while (std::getline(strm, token, ',')) {
        values.push_back(token);
      }

      MsgImuData imu;
      // Build the IMU message using SPHERES IMU data with following format:
      // VERTIGO t, SPH t, gyro x, gyro y, gyro z, accel x, accel y, accel z.
      imu.vertigo_test_time = std::stod(values[0]);
      imu.sph_test_time = std::stod(values[1]);
      imu.gyro(0) = std::stod(values[2]);
      imu.gyro(1) = std::stod(values[3]);
      imu.gyro(2) = std::stod(values[4]);
      imu.accel(0) = std::stod(values[5]);
      imu.accel(1) = std::stod(values[6]);
      imu.accel(2) = std::stod(values[7]);

      measurements.push_back(imu);
    }
  }

  file.close();

  return measurements;
}

/* ************************************************************************** */
std::vector<MsgThrusterTimes> LoadThrusterData(const std::string &thr_file) {
  // Vector with the thruster firing times.
  std::vector<MsgThrusterTimes> firing_times;

  // Variables to store text lines and words.
  std::string line, token;

  // Analyze firing times file only if it exists.
  std::ifstream file(thr_file.c_str());
  if (file.good()) {
    while (std::getline(file, line)) {
      // Parse the file, line by line.
      std::stringstream strm(line);

      // Get all line values together.
      std::vector<std::string> values;
      while (std::getline(strm, token, ',')) {
        values.push_back(token);
      }

      MsgThrusterTimes thr;
      // Build the thruster times message using the file with the format:
      // VERTIGO t, SPH t, 12 vals for on-times, 12 vals for off-times.
      thr.vertigo_test_time = std::stoul(values[0]);
      thr.sph_test_time = std::stoul(values[1]);
      for (int i = 2; i < 14; i++) thr.on_time(i - 2) = std::stoi(values[i]);
      for (int i = 14; i < 26; i++) thr.off_time(i - 14) = std::stoi(values[i]);

      firing_times.push_back(thr);
    }
  }

  file.close();

  return firing_times;
}

/* ************************************************************************** */
std::vector<RelativePoseMeasurement> LoadRelativePoseMeasurements(
    const std::string &rpose_file) {
  // Vector with the relative pose measurements.
  std::vector<RelativePoseMeasurement> rpose_meas;

  // Variables to store text lines and words.
  std::string line, token;

  // Variables for parsing.
  double tx, ty, tz;
  double v;

  std::ifstream file(rpose_file.c_str());
  if (file.good()) {
    while (std::getline(file, line)) {
      // Parse the file, line by line.
      std::stringstream strm(line);

      RelativePoseMeasurement rpm;
      // Get the times and IDs.
      strm >> rpm.time_i >> rpm.time_j >> rpm.i >> rpm.j;

      // Get the translation.
      strm >> tx >> ty >> tz;
      rpm.relative_pose.t << tx, ty, tz;

      std::vector<double> values;

      // Get the rotation.
      values.reserve(9);
      for (int i = 0; i < 9; i++) {
        strm >> v;
        values.push_back(v);
      }
      rpm.relative_pose.R = Eigen::Matrix3d(values.data());

      // Get the covariance.
      values.clear();
      values.reserve(36);
      for (int i = 0; i < 36; i++) {
        strm >> v;
        values.push_back(v);
      }
      rpm.relative_pose.cov = Eigen::Matrix<double, 6, 6>(values.data());

      // Get the number of inliers.
      strm >> rpm.inliers;

      rpose_meas.push_back(rpm);
    }
  }

  return rpose_meas;
}

/* ************************************************************************** */
// std::vector<MsgGlobalMet> LoadGlobalMetrology(const std::string &gm_file) {
//   // Vector with the IMU measurements.
//   std::vector<MsgGlobalMet> measurements;
//
//   // Variables to store text lines and words.
//   std::string line, token;
//
//   // Analyze imu file only if it exists.
//   std::ifstream file(gm_file.c_str());
//   if (file.good()) {
//     while (std::getline(file, line)) {
//       // Parse the file, line by line.
//       std::stringstream strm(line);
//
//       // Get all line values together.
//       std::vector<std::string> values;
//       while (std::getline(strm, token, ',')) {
//         values.push_back(token);
//       }
//
//       MsgGlobalMet gm;
//       // Build the GM message using SPHERES data with following format:
//       // VERTIGO t, SPH t, pos, vel, quat, omega.
//       gm.vertigo_test_time = std::stod(values[0]);
//       gm.sph_test_time = std::stoul(values[1]);
//
//       gm.t(0) = std::stod(values[2]);
//       gm.t(1) = std::stod(values[3]);
//       gm.t(2) = std::stod(values[4]);
//
//       gm.v(0) = std::stod(values[5]);
//       gm.v(1) = std::stod(values[6]);
//       gm.v(2) = std::stod(values[7]);
//
//       // Eigen quaterions are built in w,x,y,z order.
//       gm.q = Eigen::Quaterniond(std::stod(values[11]), std::stod(values[8]),
//                                 std::stod(values[9]), std::stod(values[10]));
//       gm.R = omsci::quat2rot(gm.q);
//
//       gm.omega(0) = std::stod(values[12]);
//       gm.omega(1) = std::stod(values[13]);
//       gm.omega(2) = std::stod(values[14]);
//
//       measurements.push_back(gm);
//     }
//   }
//
//   file.close();
//
//   return measurements;
// }

/* ************************************************************************** */
std::vector<MsgPosVel> LoadPosVelData(const std::string &pv_file) {
  // Vector with the IMU measurements.
  std::vector<MsgPosVel> measurements;

  // Variables to store text lines and words.
  std::string line, token;

  // Analyze imu file only if it exists.
  std::ifstream file(pv_file.c_str());
  if (file.good()) {
    while (std::getline(file, line)) {
      // Parse the file, line by line.
      std::stringstream strm(line);

      // Get all line values together.
      std::vector<std::string> values;
      while (std::getline(strm, token, ',')) {
        values.push_back(token);
      }

      MsgPosVel pv;
      // Build the posVel message using SPHERES data with following format:
      // VERTIGO t, pos, vel.
      pv.vertigo_test_time = std::stod(values[0]);

      pv.t(0) = std::stod(values[1]);
      pv.t(1) = std::stod(values[2]);
      pv.t(2) = std::stod(values[3]);

      pv.v(0) = std::stod(values[4]);
      pv.v(1) = std::stod(values[5]);
      pv.v(2) = std::stod(values[6]);

      measurements.push_back(pv);
    }
  }

  file.close();

  return measurements;
}

// /* ************************************************************************** */
// void SyncBlobToImages(const std::vector<MsgPosVel> &pv,
//                       std::vector<sph::StereoFrame> *images) {
//   size_t n = images->size();
//   size_t m = pv.size();
//
//   size_t pv_counter = 0;
//   size_t img_start = 0;
//   double time0 = images->at(0).timestamp;
//   double time1 = images->at(1).timestamp;
//
//   for (size_t j = 0; j < m; j++) {
//     double pv_time = pv[j].vertigo_test_time;
//     double diff = time0 - pv_time;
//     if (diff < 0) {
//       pv_counter = j;
//       img_start = (time1 - pv_time < 0) ? 1 : 0;
//       break;
//     }
//   }
//
//   for (size_t i = img_start; i < n; i = i + 2) {
//     images->at(i).pv = pv[pv_counter];
//     pv_counter++;
//   }
// }
//
// /* ************************************************************************** */
// void LoadImages(const std::string &config, std::vector<StereoFrame> *images,
//                 const bool equalize) {
//   // Parse the configuration file.
//   cv::FileStorage fs(config, cv::FileStorage::Mode::READ);
//   std::string dir = static_cast<std::string>(fs["image_dir"]);
//   std::string prefix = static_cast<std::string>(fs["image_name_prefix"]);
//   std::string ext = static_cast<std::string>(fs["image_extension"]);
//   int num_of_ims = static_cast<int>(fs["num_of_images"]);
//   int init_im_num = static_cast<int>(fs["initial_image_num"]);
//
//   // Determine desired image numbers.
//   std::string full_im_name;
//   int last_im_num = init_im_num + num_of_ims;
//   std::cout << "Reading imgs: " << dir << prefix << "[" << init_im_num << "-"
//             << last_im_num << "]." << ext << std::endl;
//
//   // Read in image timestamps.
//   std::ifstream ifstr_timestamps(dir + "imageTimetags.csv");
//   std::string line_val;
//   int frame_id;
//   double frame_timestamp;
//
//   // Set the size of the individual VERTIGO images.
//   int img_height = 480, img_width = 640;
//
//   *images = std::vector<StereoFrame>();
//   images->reserve(num_of_ims);
//   while (ifstr_timestamps.good()) {
//     // Format: img_num img_timestamp
//     getline(ifstr_timestamps, line_val, ',');
//     frame_id = std::stoi(line_val);
//     getline(ifstr_timestamps, line_val, '\n');
//     frame_timestamp = std::stod(line_val);
//
//     if (frame_id < init_im_num) continue;  // Ignore unused timestamp.
//     if (frame_id >= last_im_num) break;    // Loaded all desired images, exit.
//
//     // Get raw image from file.
//     full_im_name = dir + prefix + std::to_string(frame_id) + "." + ext;
//     cv::Mat im = cv::imread(full_im_name, cv::IMREAD_GRAYSCALE);
//
//     // Separate images into left and right; store as dps::StereoFrames
//     if (!(frame_id == 0 && frame_timestamp == 0)) {
//       StereoFrame frame;
//       frame.img_left = im(cv::Rect(0, 0, img_width, img_height));
//       frame.img_right = im(cv::Rect(img_width, 0, img_width, img_height));
//       if (equalize) {
//         cv::equalizeHist(frame.img_left, frame.img_left);
//         cv::equalizeHist(frame.img_right, frame.img_right);
//       }
//       frame.timestamp = frame_timestamp;
//       frame.id = frame_id;
//       images->emplace_back(frame);
//     } else {
//       std::cout << "Some weird thing happended with: " << full_im_name
//                 << std::endl;
//       std::cout << " frame id: " << frame_id << std::endl;
//       std::cout << " frame ts: " << frame_timestamp << std::endl;
//     }
//   }
// }

/* ************************************************************************** */
Eigen::MatrixXd ReadXhatFile(const std::string &filename) {
  std::vector<double> values;
  int num_rows = 0;
  std::string line, token;

  std::ifstream file(filename.c_str());
  if (file.good()) {
    while (std::getline(file, line)) {
      std::stringstream strm(line);
      while (strm >> token) {
        values.push_back(std::stod(token));
      }
      num_rows++;
    }
  }
  file.close();

  Eigen::MatrixXd xhat = Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      values.data(), num_rows, values.size() / num_rows);

  return xhat;
}

/* ************************************************************************** */
// void WriteG2oLine(std::ofstream *out, const int i, const int j,
//                   const Pose3D T_ij) {
//   // Need to rearrange the covariance matrix from [theta pos] to [pos theta].
//   Eigen::MatrixXd cov(6, 6);
//   // Set the rotation block.
//   cov.block(3, 3, 3, 3) = T_ij.cov.block(0, 0, 3, 3);
//   // Set the translation block.
//   cov.block(0, 0, 3, 3) = T_ij.cov.block(3, 3, 3, 3);
//   // Swap the cross terms.
//   cov.block(0, 3, 3, 3) = T_ij.cov.block(3, 0, 3, 3);
//   cov.block(3, 0, 3, 3) = T_ij.cov.block(0, 3, 3, 3);
//
//   // Get the information matrix to write upper triangular block to file.
//   Eigen::MatrixXd info = cov.inverse();
//
//   // Get the rotation in quaternion form.
//   Eigen::Quaterniond q = omsci::rot2quat(T_ij.R);
//
//   // Write to file.
//   *out << "EDGE_SE3:QUAT " << i << " " << j << " " << T_ij.t(0) << " "
//        << T_ij.t(1) << " " << T_ij.t(2) << " " << q.x() << " " << q.y() << " "
//        << q.z() << " " << q.w();
//   // Still not done. Have to write the upper triangular block of the info mat.
//   for (int k = 0; k < 6; k++) {
//     for (int l = k; l < 6; l++) {
//       *out << " " << info(k, l);
//     }
//   }
//   *out << std::endl;
// }

/* ************************************************************************** */
void LoadOmegas(const std::string &filename, std::vector<double> *times,
                Eigen::MatrixXd *omegas) {
  // Vector with the IMU measurements.
  std::vector<Eigen::Vector3d> omegasVec;
  times->clear();

  // Variables to store text lines and words.
  std::string line, token;

  // Variables for parsing.
  double t;

  std::ifstream file(filename.c_str());
  if (file.good()) {
    while (std::getline(file, line)) {
      // Parse the file, line by line.
      std::stringstream strm(line);

      RelativePoseMeasurement rpm;
      Eigen::Vector3d omega;
      // Get the times and meas.
      strm >> t >> omega(0) >> omega(1) >> omega(2);

      omegasVec.push_back(omega);
      times->push_back(t);
    }
  }

  file.close();

  size_t N = omegasVec.size();
  *omegas = Eigen::MatrixXd(3, N);
  for (size_t i = 0; i < N; i++) {
    (*omegas).col(i) = omegasVec[i];
  }
}

}  // namespace sph
