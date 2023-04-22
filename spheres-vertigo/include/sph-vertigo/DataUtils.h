/**
 * @file DataUtils.h
 * @brief Utility functions for interacting with the raw datasets.
 * @date September 11, 2019
 * @author MIT Space Systems Laboratory
 * @author Antonio Teran (teran@mit.edu)
 * Copyright 2020 MIT Space Systems Laboratory
 */

#ifndef SPH_VERTIGO_DATAUTILS_H_
#define SPH_VERTIGO_DATAUTILS_H_

// NOLINTNEXTLINE
#include <chrono>
#include <string>
#include <vector>

#include "sph-vertigo/CommonTypes.h"

namespace sph {

/**
 * @brief This function returns a chrono::time_point struct encoding the time at
 * which it is called.
 */
inline std::chrono::time_point<std::chrono::high_resolution_clock> tick() {
  return std::chrono::high_resolution_clock::now();
}

/**
 * @brief When this function is called with a chrono::time_point struct returned
 * by tick(), it returns the elapsed time (in seconds) between the calls to
 * tick() and tock().
 */
inline double tock(
    const std::chrono::time_point<std::chrono::high_resolution_clock>
        &tick_time) {
  auto counter = std::chrono::high_resolution_clock::now() - tick_time;
  return std::chrono::duration_cast<std::chrono::milliseconds>(counter)
             .count() /
         1000.0;
}

/**
 * @brief Parse the SPHERES IMU text files to vector of IMU messages.
 * @param imu_file Path to the IMU text file (imu_data.txt).
 * @return A vector with IMU message structs.
 */
std::vector<MsgImuData> LoadImuData(const std::string &imu_file);

/**
 * @brief Parse the SPHERES thruster firing times file to build messages.
 * @param thr_file Path to the thruster firing times text file.
 * @return A vector with thruster firing time message structs.
 */
std::vector<MsgThrusterTimes> LoadThrusterData(const std::string &thr_file);

/**
 * @brief Get the relative pose measurements for the VO backbone chain.
 */
std::vector<RelativePoseMeasurement> LoadRelativePoseMeasurements(
    const std::string &rpose_file);

/**
 * @brief Parse the SPHERES global metrology file to build messages.
 * @param gm_file  Path to the global metrology measurements text file.
 * @return A vector with global metrology message structs.
 */
// std::vector<MsgGlobalMet> LoadGlobalMetrology(const std::string &gm_file);

/**
 * @brief Parse the SPHERES blob tracking pos vel measurements.
 * @param pv_file  Path to the pos vel measurements text file.
 * @return A vector with blob track message structs.
 */
std::vector<MsgPosVel> LoadPosVelData(const std::string &pv_file);

/**
 * @brief Add the posVel messages to the corresponding stereo frame.
 * @param[in] pv          Vector with pos vel messages from blobtracking.
 * @param[in/out] images  Vector with all loaded images.
//  */
// void SyncBlobToImages(const std::vector<MsgPosVel> &pv,
//                       std::vector<sph::StereoFrame> *images);

/**
 * @brief Calculate the average timestep between a collection of measurements.
 * @param msg_vec Vector of timestamped measuremnt struct with `sph_test_time`.
 * @return The average timestep between measurements, in seconds.
 */
template <typename MsgVecT>
double AvgMsgTimestep(const MsgVecT &msg_vec);

/**
 * @brief Reads a configuration file and loads all stereo images from a dataset.
 * @param[in] config Path to configuration file in the style of an OpenCV yaml.
 * @param[out] images Vector of structures holding a stereo image pair.
 * @param[in] equalize Whether to equalize the histogram of grayscale image.
 */
// void LoadImages(const std::string &config, std::vector<StereoFrame> *images,
//                 const bool equalize = true);

/**
 * @brief Parse trajectory estimates from file.
 * @param filename Path to the file with the pose estimates.
 * @return Eigen matrix with translation and rotations block [t R].
 */
Eigen::MatrixXd ReadXhatFile(const std::string &filename);

/**
 * @brief Write a line with the relative pose measurement in g2o file format.
 * @param[in/out] out   The stream to which to write.
 * @param[in] i         Index for the base pose.
 * @param[in] j         Index for the destination pose.
 * @param[in] T_ij      The relative pose measurement.
 *
 * The g2o format specifies a 3D relative pose measurement in the
 * following form:
 *
 * EDGE_SE3:QUAT id1, id2, dx, dy, dz, dqx, dqy, dqz, dqw
 *
 * I11 I12 I13 I14 I15 I16
 *     I22 I23 I24 I25 I26
 *         I33 I34 I35 I36
 *             I44 I45 I46
 *                 I55 I56
 *                     I66
 */
// void WriteG2oLine(std::ofstream *out, const int i, const int j,
//                   const Pose3D T_ij);

/**
 * @brief Read in a timestamped file with angular velocity readings.
 * @param[in]  filename  Path to the file.
 * @param[out] times     Vector with timestamps.
 * @param[out] omegas    3xN matrix with the N measurements.
 */
void LoadOmegas(const std::string &filename, std::vector<double> *times,
                Eigen::MatrixXd *omegas);

}  // namespace sph

#include "sph-vertigo/DataUtils.tpp"

#endif  // SPH_VERTIGO_DATAUTILS_H_
