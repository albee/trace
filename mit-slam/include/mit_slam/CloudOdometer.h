/**
 * @file CloudOdometer.h
 * @brief Object for computing relative transformations between point clouds.
 * @date Dec 30, 2020
 * @authors tonio terán (teran@mit.edu), charles oestreich (coestrei@mit.edu), jessica todd (jetodd@mit.edu)
 * @Copyright 2020 Space Systems Laboratory, MIT
 */

#ifndef MIT_SLAM_CLOUDODOMETER_H_
#define MIT_SLAM_CLOUDODOMETER_H_

#include <ros/ros.h>
#include <unistd.h>
#include <algorithm>
#include <chrono>
#include <memory>
#include <random>

#include "teaser/registration.h"
#include "teaser/matcher.h"

#include "mit_slam/GeometryUtils.h"

// #include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/registration/icp.h>

namespace mit_slam {

/// 3D Point cloud-based odometer.
/*! This class uses depth information of the target object in the form of point
  clouds to compute a motion estimate between two measurement time steps. This
  estimate is to be used within a factor graph framework to obtain an
  optimizable pose graph.
 */
class CloudOdometer {
 public:
  /// Structure for bundling the `CloudOdometer`s parameters.
  struct Params {
    /// Robot namespace
    std::string sim = "false";
    /// Verbose console output setting for timing
    bool timing_verbose;

    // Parameter struct for TEASER++.
    teaser::RobustRegistrationSolver::Params teaser_params;

    // 3D feature detection thresholds
    double norm_radius;
    double fpfh_radius;
    double norm_scale;
    double fpfh_scale;
    double min_norm_radius;
    double min_fpfh_radius;

    // 3D feature matching settings
    bool use_absolute_scale;
    bool use_crosscheck;
    bool use_tuple_test;
    double tuple_scale;

    // ICP params
    int max_iterations;
    double trans_epsilon;
    double max_corr_dist;
    double eucl_fit_epsilon;
    double ransac_thresh;
  };
  /// Internal copy of the parameters.
  Params params_;

  /// Constructor needs to explicitly choose whether to enable ROS components.
  explicit CloudOdometer(Params params);
  /// Default destructor.
  ~CloudOdometer();

  /// Detects 3-D features in a point cloud
  teaser::FPFHCloudPtr DetectFeatures(teaser::PointCloud& cloud);

  /// Searches for 3-D feature matches between two point clouds/feature sets
  std::vector<std::pair<int, int>> MatchFeatures(
      teaser::PointCloud& src_cloud, teaser::PointCloud& tgt_cloud,
      teaser::FPFHCloudPtr& src_descriptors,
      teaser::FPFHCloudPtr& tgt_descriptors);

  /// Solves the registration problem and returns the rigid body transformation
  /// from `src`'s to `tgt`'s frame of reference, that is, the transformation
  /// `tfm` such that `tgt` = `src` \oplus `tfm` (vars in SE(d)).
  /// Also returns point cloud correspondences that were used
  Eigen::Matrix4f Register(teaser::PointCloud& src_cloud,
                           teaser::PointCloud& tgt_cloud,
                           std::vector<std::pair<int, int>> correspondences);

  Eigen::Matrix4f RegisterRawClouds(Eigen::MatrixXd src,
                                                   Eigen::MatrixXd tgt);

// Eigen::Matrix4f RegisterICP(Eigen::MatrixXf src, Eigen::MatrixXf tgt);

 private:
  std::unique_ptr<teaser::RobustRegistrationSolver> teaser_solver_ = nullptr;
};

}  // namespace mit_slam

#endif  // MIT_SLAM_CLOUDODOMETER_H_
