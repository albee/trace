/**
 * @file BlobTracker.cpp
 * @brief Object for tracking blobs inside 3D point clouds.
 * @date Dec 30, 2020
 * @authors tonio terán (teran@mit.edu), charles oestreich (coestrei@mit.edu), jessica todd (jetodd@mit.edu)
 * @Copyright 2020 Space Systems Laboratory, MIT
 */

#include "mit_slam/BlobTracker.h"

namespace mit_slam {

BlobTracker::BlobTracker(const BlobTracker::Params& params) : params_(params) {}

BlobTracker::~BlobTracker() {}

std::pair<Eigen::Vector3f, Eigen::MatrixXf>
BlobTracker::Centroid(const sensor_msgs::PointCloud2& pcd) {
  std::cout << "Centroid calculation...\n";
  // Variables to track centroid, pointer to data, and useful data points.
  Eigen::Vector3f centroid{0.0, 0.0, 0.0};
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(pcd, "x");
  int inliers = 0, total = 0;

  // Pre-allocate matrix with enough memory.
  Eigen::MatrixXf blob_pcd(3, pcd.width * pcd.height);

  // Loop over all points to get the centroid [m].
  for (; iter_x != iter_x.end(); ++iter_x) {
    Eigen::Vector3f point{iter_x[0], iter_x[1], iter_x[2]};

    // Discard points that are all zero, or too far away.
    float distance = point.norm();

    float max_x_distance;
    float max_y_distance;
    float max_z_distance;
    if (params_.ground) {
        max_x_distance = params_.max_x_distance_ground;
        max_y_distance = params_.max_y_distance_ground;
        max_z_distance = params_.max_z_distance_ground;
    }
    else {
        max_x_distance = params_.max_x_distance_iss;
        max_y_distance = params_.max_y_distance_iss;
        max_z_distance = params_.max_z_distance_iss;
    }

    if (distance != 0 && (std::abs(point(0)) <= max_x_distance &&
                          std::abs(point(1)) <= max_y_distance &&
                          std::abs(point(2)) <= max_z_distance)) {
      // Add point to the Eigen point cloud.
      blob_pcd.col(inliers) = point;

      // Add information to centroid.
      centroid += point;
      inliers++;
    }
    total++;
  }

  Eigen::MatrixXf trunc_pcd;
  trunc_pcd = blob_pcd.block(0, 0, 3, inliers);
  Eigen::Vector3f final_centroid;
  bool pcd_empty = false;
  if (trunc_pcd.rows() == 0) {
    std::cout << "[MIT-SLAM]: Truncated point cloud empty." << std::endl;
    pcd_empty = true;
    final_centroid = centroid;
  }
  else {
    final_centroid = centroid / inliers;
  }

  // Adjust truncated distance based on centroid estimate
  if (!pcd_empty) {
    if (params_.ground) {
      params_.max_z_distance_ground = final_centroid(2) + params_.centroid_add_ground;
    }
    else {
      params_.max_z_distance_iss = final_centroid(2) + params_.centroid_add_iss;
    }
  }

  // Downsample the point cloud if there are a lot of points (speeds up teaser)
  Eigen::MatrixXf final_pcd;
  if (!pcd_empty) {
    if (trunc_pcd.cols() > params_.max_pcd_size) {
      final_pcd = BlobTracker::DownSamplePcd(trunc_pcd);
    }
    else {
      final_pcd = trunc_pcd;
    }
  }
  else {
    final_pcd = trunc_pcd;
  }

  // Return centroid and the Eigen point cloud.
  return std::make_pair(final_centroid, final_pcd);
}

Eigen::MatrixXf BlobTracker::DownSamplePcd(const Eigen::MatrixXf &raw_pcd) {
  // 1. Convert Eigen raw to PCL raw
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = 0; i < raw_pcd.cols(); i++) {
    pcl::PointXYZ temp;
    temp.x = raw_pcd(0, i);
    temp.y = raw_pcd(1, i);
    temp.z = raw_pcd(2, i);
    pcl_raw_cloud->points.push_back(temp);
  }

  // 2. Downsample point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> grid;
  grid.setInputCloud(pcl_raw_cloud);
  grid.setLeafSize(params_.leaf_size_x, params_.leaf_size_y, params_.leaf_size_z);
  grid.filter(*pcl_downsampled_cloud);

  // 3. Convert back to Eigen
  int size_pcl = pcl_downsampled_cloud->size();
  Eigen::Matrix<float, 3, Eigen::Dynamic> downsampled_pcd;
  downsampled_pcd.resize(3, size_pcl);
  for (int i = 0; i < size_pcl; i++) {
    pcl::PointXYZ temp;
    temp = pcl_downsampled_cloud->points[i];
    downsampled_pcd(0,i) = temp.x;
    downsampled_pcd(1,i) = temp.y;
    downsampled_pcd(2,i) = temp.z;
  }

  return downsampled_pcd;
}

}  // namespace slam
