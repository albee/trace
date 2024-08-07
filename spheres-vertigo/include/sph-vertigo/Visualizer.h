/**
 * @file Visualizer.h
 * @brief Simple multi-threaded 3D graph visualization based on Pangolin.
 * @date June 02, 2020
 * @author Tonio Teran, teran@mit.edu
 * Copyright 2020 tonio
 */

#ifndef SPH_VERTIGO_VISUALIZER_H_
#define SPH_VERTIGO_VISUALIZER_H_

#include <pangolin/pangolin.h>
#include <pangolin/scene/axis.h>
#include <pangolin/scene/scenehandler.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>

#include <Eigen/Dense>
#include <tuple>
#include <vector>

#include "sph-vertigo/GeometryUtils.h"

namespace sph {

/// Trajectory consisiting of vector of Eigen-aligned 4x4 SE(3) matrices.
typedef std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>
    Trajectory3;
/// 3D Pose with axes length (1st double) and line width (2nd double) for viz.
typedef std::tuple<Eigen::Matrix4d, double, double> VizPose;
/// Stereo frame with triangulated points and visualization properties, with the
/// doubles representing: r, g, b, point size.
typedef std::tuple<omsci::StereoFrame, double, double, double, double> VizFrame;

/**
 * @class Visualizer
 * @brief Class for wrapping OpenGL and Pangoling to visualize in 3D.
 */
class Visualizer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /**
   * @brief Default constructor.
   * @param w Width of the screen [px].
   * @param h Height of the screen [px].
   * @param f Focal distance of the visualization camera [px].
   */
  Visualizer(const float w = 1200.0f, const float h = 800.0f,
             const float f = 500.0f);

  /**
   * @brief Default destructor.
   */
  ~Visualizer();

  /**
   * @brief Main visualization for Simple3dWorld that does all the drawing.
   */
  void RenderWorld();

  /**
   * @brief Estimates and corresponding time stamps for 3D visualization.
   * @param[in] values  System estimates.
   * @param[in] times   Corresponding timestamps.
   */
  void UpdateEstimate(const gtsam::Values& values,
                      const std::vector<double>& times);

  /**
   * @brief Add a visualization pose element.
   * @param[in] vpose   Visualization tuple with pose, axes length, and width.
   */
  void AddVizPose(const VizPose& vpose) { vposes_.push_back(vpose); }

  /**
   * @brief Add a visualization pose element.
   * @param[in] pose     3D pose of triad to visualize.
   * @param[in] length   Length of the pose axes.
   * @param[in] width    Width of the pose axes..
   */
  void AddVizPose(const Eigen::Matrix4d& pose, const double length,
                  const double width) {
    AddVizPose(std::make_tuple(pose, length, width));
  }

  /**
   * @brief Add triangulated stereo frame in order to visualize points.
   * @param[in] vframe  Triangulated stereo frame with viz properties.
   */
  void AddVizFrame(const VizFrame& vframe) { vframes_.push_back(vframe); }

  /**
   * @brief Add triangulated stereo frame in order to visualize points.
   * @param[in] frame     Triangulated stereo frame with points.
   * @param[in] r         Normalized red color value.
   * @param[in] g         Normalized green color value.
   * @param[in] b         Normalized blue color value.
   * @param[in] size      Size of points.
   */
  void AddVizFrame(const omsci::StereoFrame& frame, const double r,
                   const double g, const double b, const double size) {
    AddVizFrame(std::make_tuple(frame, r, g, b, size));
  }

  /**
   * @brief Add a matched stereo frame for visualization.
   * @param matched_frame  OpenCV matched stereo frame.
   */
  void AddMatchedFrame(const cv::Mat& matched_frame) {
    cv::Mat matched_ushort;
    matched_frame.convertTo(matched_ushort, CV_8UC3);
    cv::flip(matched_ushort.clone(), matched_, 0);
  }

  inline void AddOmega(const Eigen::Vector3d& omega) {
    omegas_.push_back(omega);
  }

  inline void AddTargetPose(const Eigen::Matrix4d& p) { tgt_.push_back(p); }

  /**
   */
  inline void SetXYProjection(const Eigen::MatrixXd& pts) {
    xyproj_.clear();
    for (size_t i = 0; i < pts.cols(); i++) {
      xyproj_.push_back(Eigen::Vector3d{pts(0, i), pts(1, i), 0.0});
    }
  }

  /**
   */
  inline void SetXZProjection(const Eigen::MatrixXd& pts) {
    xzproj_.clear();
    for (size_t i = 0; i < pts.cols(); i++) {
      xzproj_.push_back(Eigen::Vector3d{pts(0, i), 0.0, pts(1, i)});
    }
  }

  /**
   */
  inline void SetYZProjection(const Eigen::MatrixXd& pts) {
    yzproj_.clear();
    for (size_t i = 0; i < pts.cols(); i++) {
      yzproj_.push_back(Eigen::Vector3d{0.0, pts(0, i), pts(1, i)});
    }
  }

 private:
  /**
   * @brief Renders a world consisting of poses and landmarks.
   * @param[in] trajectory Eigen-aligned vector of 3D poses.
   * @param[in] landmarks Vector of 3D points.
   */
  void DrawWorld(const Trajectory3& trajectory,
                 const std::vector<Eigen::Vector3d>& landmarks) const;

  /**
   * @brief Renders the trajectory as a sequence of triads.
   * @param[in] trajectory Eigen-aligned vector of 3D poses.
   */
  void DrawTrajectory(const Trajectory3& trajectory,
                      const double axesLength = 0.2) const;

  void DrawObserver() const;
  void DrawTarget() const;
  void DrawGyroPolhode() const;
  void DrawPolhodePlaneProjections() const;

  /**
   * @brief Draw the points from the available frames.
   * @param[in] vframes  Triangulated stereo frames with viz properties.
   */
  void DrawFrames(const std::vector<VizFrame> vframes) const;

  float w_;  ///< Starting width of viewer window [px].
  float h_;  ///< Starting height of viewer window [px].
  float f_;  ///< Focal length of viewer camera [px].

  // Manually-modifiable variables.
  std::vector<VizPose> vposes_;    ///< Manually added poses to visualize.
  std::vector<VizFrame> vframes_;  ///< Processed frames w/ points to visualize.

  // OpenCV for matched frames.
  cv::Mat matched_;  ///< Most recent matched stereo frame.

  // GTSAM estimates.
  gtsam::Values vals_;         ///< Current system estimates.
  std::vector<double> times_;  ///< Corresponding estimate timestamps.
  Trajectory3 est_;            ///< Current state estimate trajectory.
  Trajectory3 tgt_;            ///< Current trajectory estimate for the target.
  Trajectory3 estWrtG_;        ///< Current inspector estimate in G frame.
  std::vector<Eigen::Vector3d> omegas_;     ///< Tgt's ngular velocity history.
  std::vector<Eigen::Vector3d> rotomegas_;  ///< Tgt's aligned omegas.
  std::vector<Eigen::Vector3d> xyproj_;     ///< Polhode's XY projection.
  std::vector<Eigen::Vector3d> xzproj_;     ///< Polhode's XZ projection.
  std::vector<Eigen::Vector3d> yzproj_;     ///< Polhode's YZ projection.
  Eigen::Vector3d CGpos_;                   ///< Center of mass position.
  Eigen::Matrix3d R_BG_;                    ///< Principal axes orientation.
  std::vector<Eigen::Vector3d> gyros_;      ///< Ground truth omegas.
  std::vector<double> gyrotimes_;           ///< Time stamps for gyro.

  // 2D Plots.
  pangolin::DataLog omegaxLog_;
};

}  // namespace sph

#endif  // SPH_VERTIGO_VISUALIZER_H_
