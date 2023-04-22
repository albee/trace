/**
 * @file BlobTracker.h
 * @brief Object for tracking blobs inside 3D point clouds.
 * @date Dec 30, 2020
 * @authors tonio terán (teran@mit.edu), charles oestreich (coestrei@mit.edu), jessica todd (jetodd@mit.edu)
 * @Copyright 2020 Space Systems Laboratory, MIT
 */

#ifndef MIT_SLAM_BLOBTRACKER_H_
#define MIT_SLAM_BLOBTRACKER_H_

#include <ros/ros.h>
#include <Eigen/Dense>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/Marker.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <string>
#include <utility>
#include <vector>
#include <random>
#include <chrono>

namespace mit_slam {

/// 3D Point cloud-based blob tracker class.
/*!
  Astrobee's HazCam produces 3D point clouds that can be used for state
  estimation. When observing an unknown tumbling target, it is advantageous to
  try and obtain direct measurements to its center of mass. This class offers
  geometric methods to extract such information.
 */
class BlobTracker {
 public:
  /// Structure for bundling the `BlobTracker`s parameters.
  struct Params {
    /// Robot namespace
    std::string sim = "false";
    /// Verbose console output setting for timing
    bool timing_verbose;
    /// ISS or ground test session
    bool ground;

    /// Maximum distance for a point cloud point to be considered [m].
    float max_x_distance_iss;
    float max_y_distance_iss;
    float max_z_distance_iss;
    float max_x_distance_ground;
    float max_y_distance_ground;
    float max_z_distance_ground;

    float centroid_add_ground;
    float centroid_add_iss;

    /// Max point cloud size
    int max_pcd_size;

    /// Leaf sizes for voxel grid filtering
    float leaf_size_x;
    float leaf_size_y;
    float leaf_size_z;
  };

  /// Internal copy of the parameters.
  Params params_;

  /// Constructor needs to explicitly choose whether to enable ROS components.
  explicit BlobTracker(const Params &params);
  /// Default destructor.
  ~BlobTracker();

  /// Calculates the centroid and its corresponding point cloud vector.
  std::pair<Eigen::Vector3f, Eigen::MatrixXf> Centroid(
      const sensor_msgs::PointCloud2 &pcd);

  /// Downsamples point cloud if it's too large
  Eigen::MatrixXf DownSamplePcd(const Eigen::MatrixXf &raw_pcd);

 private:
  /// The most recent point cloud centroid estimate.
  Eigen::Vector3f centroid_{0.0, 0.0, 0.0};
};

}  // namespace mit_slam

#endif  // MIT_SLAM_BLOBTRACKER_H_
