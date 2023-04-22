/**
 * @file GraphManager.h
 * @brief Class for dealing with inertial measurements.
 * @date August 01, 2020
 * @author Tonio Teran, teran@mit.edu
 * Copyright 2020 MIT Space Systems Laboratory
 */

#ifndef SPH_VERTIGO_GRAPHMANAGER_H_
#define SPH_VERTIGO_GRAPHMANAGER_H_

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <Eigen/Dense>
#include <fstream>
#include <memory>
#include <queue>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

#include "sph-vertigo/BearingRangeFactorWithTransform.h"
#include "sph-vertigo/BetweenMomentumFactor.h"
#include "sph-vertigo/CommonTypes.h"
#include "sph-vertigo/ConicProjectionFactor.h"
#include "sph-vertigo/DataUtils.h"
#include "sph-vertigo/FrontEnd.h"
#include "sph-vertigo/GeometryUtils.h"
#include "sph-vertigo/InertiaRatiosManifoldFactor.h"
#include "sph-vertigo/RotationKinematicFactor.h"

namespace sph {

/**
 * @brief Initialization parameters for GraphManager.
 */
struct GraphParams {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // - Initial parameters.
  gtsam::NavState x0{};  ///< Initial inspector's position and velocity.
  gtsam::imuBias::ConstantBias bias0{
      gtsam::Vector6::Zero()};  ///< Initial IMU bias.

  // - Noise parameters.
  double prior_pos_sigma = 0.01;   ///< Stddev for position prior [m].
  double prior_rot_sigma = 0.001;  ///< Stddev for position prior [rad].

  // - Combined IMU factor parameters.
  /// GTSAM's IMU Preintegration parameters.
  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params>
      pimParams =
          gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(0.0);

  /// Def: assumed range bias as centroid of heimspherical shell with r 0.1 [m].
  double blob_range_bias = 0.5 * 0.1;

  // - Inference parameters.
  gtsam::ISAM2Params isam2Params;  ///< Bayes tree parameters for iSAM.
  size_t opt_iters = 3;            ///< Gauss-Newton iterations per step.
  size_t max_lc_attempts = 3;      ///< Maximum number of loop closure attempts.
  size_t min_lc_inliers = 15;      ///< Min number of inliers for loop closure.

  bool show_images = false;  ///< Toggle showing proccessed images on/off.
};

/**
 * @class GraphManager
 * @brief Handle sensor messages to run GTSAM-based backend inference.
 *
 * Keeps track of the entire problem's factor graph and Bayes tree. States
 * include the inspector's navigation state and IMU bias (symbols T, v, and b,
 * respectively). The visual odometry pose graph chain involves inspector poses
 * wrt the geometric frame (symbols G and t). An additional variable is included
 * to estimate the location of the target's center of mass (symbol l).
 */
class GraphManager {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Default constructor.
   */
  GraphManager(std::shared_ptr<FrontEnd> fe,
               const GraphParams &params = GraphParams());

  /**
   * @brief Default destructor.
   */
  ~GraphManager();

  /**
   * @brief Preintegrates IMU measurement.
   * @param accel  Vector with accelerometer measurements.
   * @param omega  Vector with gyroscope measurements.
   * @param dt     Period between IMU measurements.
   */
  void AddImuMeasurement(const Eigen::Vector3d &accel,
                         const Eigen::Vector3d &omega, const double dt);

  /**
   * @brief Append relative pose measurement between two variables.
   * @param rpm  Time stamped relative pose measurement.
   */
  void AddRelativePose(const RelativePoseMeasurement &rpm);

  /**
   * @brief Store global metrology measurement in buffer.
   * @param gm  Time stamped global metrology message.
   */
  void AddGlobalMet(const MsgGlobalMet &gm);

  /**
   * @brief Add image-based keframe to graph.
   * @param frame  Time stamped stereo frame.
   */
  void AddImage(const sph::StereoFrame &frame);

  /**
   * @brief Retrive the current graph estimate.
   */
  gtsam::Values GetEstimate() const { return estimate_; }

  /**
   * @brief Get vector of time stamps corresponding to states.
   */
  std::vector<double> GetTimes() const { return times_; }

  /**
   * @brief Get the latest matched stereo frame.
   * @param[out] matched_frame  OpenCV Mat with matched stereo frame.
   */
  cv::Mat GetMatchedFrame() const { return matched_; }

  /**
   * @brief Export the current factor graph to graphviz output.
   * @param[in] out_file  Path to the filename in which to save output.
   */
  void GraphvizGraph(const std::string &out_file = "fg.dot") const;

  /**
   * @brief Export the current Bayes tree to graphviz output.
   * @param[in] out_file  Path to the filename in which to save output.
   */
  void GraphvizTree(const std::string &out_file = "bt.dot") const;

 private:
  /**
   * @brief Initialize the esitmator using the stereo frame's ID.
   * @param frame  Time stamped stereo frame.
   */
  void InitFactorGraph(const sph::StereoFrame &frame);

  /**
   * @brief Initialize geometric pose chain from visual odometry data.
   * @param[in] T_G0B0  Initial transform between geometric and body frame.
   * @param[in] T_WB0   Initial inspector pose wrt inertial frame.
   */
  void InitGeomChain(const Pose3D &T_G0B0, const Pose3D &T_WB0);

  /**
   * @brief Add IMU, visual odometry, and rotation kinematic factors.
   * @param[in] T_BiBj  Visual odometric relative pose measurement, body frame.
   * @param[in] frame   Raw stero pair image information.
   *
   * Inertial information is contained within the member variable `odometer_`.
   */
  void AddFactors(const Pose3D &T_BiBj, const sph::StereoFrame &frame);

  /**
   * @brief Choose a random state in the past and try to match stereo frames.
   * @param processed_frame_j  Frame with triangulated features.
   */
  void AttemptLoopClosure(const omsci::StereoFrame &processed_frame_j,
                          const sph::StereoFrame &frame);

  void ExportFullG2oFile() const;

  void ExportAngularVelocities(const std::string &filename) const;

  void ExportInertiaRatioParallel(const double time) const;

  // Objects.
  std::shared_ptr<FrontEnd> fe_ = nullptr;  ///< Front-end manager object.
  std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> odometer_ =
      nullptr;  ///< Odometer for IMU preintegration between keyframes.
  std::shared_ptr<gtsam::ISAM2> isam_ = nullptr;  ///< Bayes tree for iSAM.
  gtsam::noiseModel::Diagonal::shared_ptr blobNoiseModel_ = nullptr;
  boost::shared_ptr<gtsam::ConicProjectionFactor> conicFactor_ = nullptr;
  std::default_random_engine gen_;

  GraphParams params_;  ///< Internal copy of the initialization parameters.

  // Buffers.
  std::vector<MsgGlobalMet> gm_;          ///< All global metrology meas.
  std::vector<MsgGlobalMet> corresp_gm_;  ///< State-corresponding GM.
  std::vector<omsci::StereoFrame> processed_frames_;  ///< Processed images.
  std::vector<sph::StereoFrame> raw_frames_;          ///< Raw images.
  std::vector<double> times_;                         ///< State estimate times.
  std::vector<Pose3D> geometric_poses_;               ///< Poses from VO.
  std::unordered_map<int, int> imgIdToState_;  ///< Hash to map img ID to state.
  std::queue<gtsam::InertiaRatiosManifoldFactor> irfactors_;  ///< Accumulate.
  std::queue<gtsam::Vector3> hbuffer_;  ///< Angular momentums.
  std::queue<gtsam::Symbol> hkeys_;     ///< GTSAM Symbols.
  std::queue<gtsam::BetweenMomentumFactor>
      hbiasbuffer_;                      ///< Angular momentums bias.
  std::queue<gtsam::Symbol> hbiaskeys_;  ///< Angmom bias GTSAM Symbols.
  bool initH_ = false;
  gtsam::Vector3 initHvalue_;
  size_t initHiD_ = 0;
  gtsam::Symbol hLatestKey_;

  // Flags.
  bool fgReady_ = false;              ///< True only after receiving 1st image.
  bool geometricChainReady_ = false;  ///< True only after processing 1st pair.
  bool hFirstPass_ = true;            ///< Non angular momentum recorded yet.

  // Coordinate frames.
  Pose3D T_G0B0_;  ///< Initial value for the geometric frame.
  Pose3D T_BC_;    ///< Pose of left camera in inspector body frame.
  Pose3D T_GB_;    ///< Current value for the geometric frame.

  // Counters.
  int curStateId_ = 0;  ///< Counter to keep state index.

  // Outputs.
  gtsam::Values estimate_;  ///< Current smoothed system estimate.
  cv::Mat matched_;         ///< Most recent matched stereo frame.
  std::ofstream pao_;       ///< Output stream for principal axes optimized val.
  std::ofstream tree_;      ///< Output stream for tree graphs props.
  std::ofstream g2o_;       ///< Output stream for g2o file.
  std::ofstream imu_;       ///< Output stream for IMU only g2o file.
  std::ofstream combined_;  ///< Combined IMU and VO measurements for g2o.
  std::ofstream perf_;      ///< Output stream for performance metrics.
  std::ofstream ratios_;    ///< Output stream for the inertia ratios.
  std::ofstream momentum_;  ///< Output stream for the angular momentum.
  mutable std::ofstream jpar_;  ///< Output stream for inertia ratios parallel.
};

}  // namespace sph

#endif  // SPH_VERTIGO_GRAPHMANAGER_H_
