/**
 * @file FrontEnd.h
 * @brief Class for dealing with sensor measurements.
 * @date August 01, 2020
 * @author Tonio Teran, teran@mit.edu
 * Copyright 2020 MIT Space Systems Laboratory
 */

#ifndef SPH_VERTIGO_FRONTEND_H_
#define SPH_VERTIGO_FRONTEND_H_

#include <memory>
#include <string>

#include "sph-vertigo/CommonTypes.h"
#include "sph-vertigo/FeatureHandler.h"
#include "sph-vertigo/ImageUtils.h"
#include "sph-vertigo/Rectifier.h"
#include "sph-vertigo/StereoDepthSensor.h"
#include "sph-vertigo/VisualOdometer.h"

namespace sph {

/**
 * @brief Initialization parameters for GraphManager.
 */
struct FrontEndParams {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // SPHERES parameters.
  // clang-format off
  /// Pose of the left VERTIGO camera frame wrt SPHERES body frame.
  Eigen::Matrix4d T_BC = (Eigen::MatrixXd(4, 4) <<
                          0,  0,  1, 0.234 - 0.025,
                         -1,  0,  0,         0.045,
                          0, -1,  0,         0.030,
                          0,  0,  0,         1).finished();
  // clang-format on

  // Dataset parameters.
  std::string dataset_path = "/Users/tonio/data/vertigomod/TS53T7R3mod/";

  // Scene parameters.
  double min_depth = 0.1;  ///< Minimum target depth [m].
  double max_depth = 1.1;  ///< Maximum target depth [m].

  // Stereo depth sensor parameters (block matching).
  int num_disp = 16 * 10;  ///< OpenCV BlockMatcher.
  int block_size = 21;     ///< OpenCV BlockMatcher.

  // Feature handler parameters.
  int detectorExtractorMatcherTypes[3] = {omsci::FDT_SURF, omsci::DET_SURF,
                                          omsci::DMT_BruteForce};
  double hessianThresh = 400;  ///< For SURF only.
};

/**
 * @class FrontEnd
 * @brief Extract back-end information from sensor measurements.
 */
class FrontEnd {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Default constructor.
   */
  explicit FrontEnd(const FrontEndParams &params = FrontEndParams());

  /**
   * @brief Default destructor.
   */
  ~FrontEnd();

  /**
   * @brief Extract features, triangulate, and assign covariances.
   * @param[in]  frame            Stereo frame with raw information.
   * @param[out] processed_frame  Triangulated stereo frame.
   *
   * Note the two different types of `StereFrame`s.
   */
  void ProcessImage(const sph::StereoFrame &frame,
                    omsci::StereoFrame *processed_frame) const;

  /**
   * @brief 4way matching to get inlier set and relative pose (visual odometry).
   * @param[in]  processed_frame_i  Triangulated stereo frame at time i.
   * @param[in]  processed_frame_j  Triangulated stereo frame at time i.
   * @param[out] inlier_frame_i     Frame at i with only inlier information.
   * @param[out] inlier_frame_j     Frame at j with only inlier information.
   * @param[out] poseCitoCj         Relative pose measurement from i to j.
   */
  void MatchPair(const omsci::StereoFrame &processed_frame_i,
                 const omsci::StereoFrame &processed_frame_j,
                 omsci::StereoFrame *inlier_frame_i,
                 omsci::StereoFrame *inlier_frame_j, Pose3D *poseCitoCj) const;

  /**
   * @brief Draw matched stereo frames onto a CV image.
   * @param[in]  frame_j            Stereo frame with raw information.
   * @param[in]  processed_frame_j  Triangulated stereo frame at time i.
   * @param[in]  inlier_frame_i     Frame at i with only inlier information.
   * @param[in]  inlier_frame_j     Frame at j with only inlier information.
   * @param[out] out                CV image with drawn matches.
   */
  void DrawMatched(const sph::StereoFrame &frame_j,
                   const omsci::StereoFrame &processed_frame_j,
                   const omsci::StereoFrame &inlier_frame_i,
                   const omsci::StereoFrame &inlier_frame_j,
                   cv::Mat *out) const;
  /**
   * @brief Only draw matches between stereo frames onto a CV image.
   * @param[in]  frame_j            Stereo frame with raw information.
   * @param[in]  processed_frame_j  Triangulated stereo frame at time i.
   * @param[in]  inlier_frame_i     Frame at i with only inlier information.
   * @param[in]  inlier_frame_j     Frame at j with only inlier information.
   * @param[in]  rgbLR              Color for the left to right matches.
   * @param[in]  rgbAB              Color for the i to j matches.
   * @param[out] out                CV image with drawn matches.
   */
  void OnlyDrawMatches(const sph::StereoFrame &frame_j,
                       const omsci::StereoFrame &processed_frame_j,
                       const omsci::StereoFrame &inlier_frame_i,
                       const omsci::StereoFrame &inlier_frame_j,
                       const cv::Scalar &rgbLR, const cv::Scalar &rgbAB,
                       cv::Mat *out) const;

  /**
   * @brief Get the geometric frame corresponding to a stereo frame.
   * @param[in]  inlier_frame  Matched stereo frame with inlier information.
   * @param[out] T_GB   Pose of the body with respect to the geometric frame.
   *
   * Geometric frame has its origin in the centroid of all inlier features, and
   * the same orientation as the body frame at that specific time.
   */
  void GeometricFrame(const omsci::StereoFrame &inlier_frame,
                      sph::Pose3D *T_GB) const;

  /**
   * @brief Return the SE(3) transformation between body and camera.
   */
  inline sph::Pose3D T_BC() const { return T_BC_; }

 private:
  FrontEndParams params_;                           ///< Local copy of params.
  std::shared_ptr<Rectifier> rectifier_ = nullptr;  ///< To deal w/ calibration.
  std::shared_ptr<StereoCamera> cam_ = nullptr;     ///< Camer w/ calib params.
  std::shared_ptr<StereoDepthSensor> depth_ = nullptr;  ///< Depth sensor.
  std::shared_ptr<FeatureHandler> fh_ = nullptr;
  std::shared_ptr<VisualOdometer> vo_ = nullptr;
  // REFACTOR.
  cv::FeatureDetector *detector_ = nullptr;       ///< For feature handler.
  cv::DescriptorExtractor *extractor_ = nullptr;  ///< For feature handler.
  cv::DescriptorMatcher *matcher_ = nullptr;      ///< For feature handler.

  // Constants.
  double min_disparity_ = 0.0;    ///< Minimum value wrt calib params [px].
  double max_disparity_ = 255.0;  ///< Maximum value wrt calib params [px].
  sph::Pose3D T_BC_;              ///< Pose of left camera in body frame.
};

}  // namespace sph

#endif  // SPH_VERTIGO_FRONTEND_H_
