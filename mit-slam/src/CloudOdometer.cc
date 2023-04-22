/**
 * @file CloudOdometer.cpp
 * @brief Object for computing relative transformations between point clouds.
 * @date Nov 18, 2020
 * @authors tonio terán (teran@mit.edu), charles oestreich (coestrei@mit.edu), jessica todd (jetodd@mit.edu)
 * @Copyright 2020 Space Systems Laboratory, MIT
 */

#include "mit_slam/CloudOdometer.h"

namespace mit_slam {

CloudOdometer::CloudOdometer(CloudOdometer::Params params) : params_(params),
      teaser_solver_(std::unique_ptr<teaser::RobustRegistrationSolver>(new teaser::RobustRegistrationSolver(std::forward<teaser::RobustRegistrationSolver::Params>(params.teaser_params)))) {}

//teaser_solver_(std::make_unique<teaser::RobustRegistrationSolver>(params.teaser_params)) {}
CloudOdometer::~CloudOdometer() {}

teaser::FPFHCloudPtr CloudOdometer::DetectFeatures(teaser::PointCloud& cloud) {
  teaser::FPFHEstimation fpfh;
  teaser::FPFHCloudPtr features = fpfh.computeFPFHFeatures(cloud,
                                                           params_.norm_radius,
                                                           params_.fpfh_radius);


  return features;
}

std::vector<std::pair<int,int>> CloudOdometer::MatchFeatures(teaser::PointCloud& src_cloud,
                                                             teaser::PointCloud& tgt_cloud,
                                                             teaser::FPFHCloudPtr& src_descriptors,
                                                             teaser::FPFHCloudPtr& tgt_descriptors) {
  teaser::Matcher matcher;
  auto correspondences = matcher.calculateCorrespondences(src_cloud, tgt_cloud, *src_descriptors, *tgt_descriptors,
                                                          params_.use_absolute_scale, params_.use_crosscheck, params_.use_tuple_test, params_.tuple_scale);


  return correspondences;
}

Eigen::Matrix4f CloudOdometer::Register(teaser::PointCloud& src_cloud,
                                        teaser::PointCloud& tgt_cloud,
                                        std::vector<std::pair<int,int>> correspondences) {
  teaser_solver_->reset(params_.teaser_params);
  Eigen::Matrix4f tfm;

  teaser::RegistrationSolution result = teaser_solver_->solve(src_cloud, tgt_cloud, correspondences);

  // Parse rotation and translation solutions.
  tfm.block(0, 0, 3, 3) = result.rotation.cast<float>();
  tfm.block(0, 3, 3, 1) = result.translation.cast<float>();
  tfm(3, 3) = 1;  // Homogeneous (SE(3)).
  Eigen::Matrix4f tfm_output = fixNumerics(tfm);

  return tfm_output;
}

Eigen::Matrix4f CloudOdometer::RegisterRawClouds(Eigen::MatrixXd src,
                                                 Eigen::MatrixXd tgt) {
  teaser_solver_->reset(params_.teaser_params);
  Eigen::Matrix4f tfm;

  teaser::RegistrationSolution result = teaser_solver_->solve(src, tgt);

  // Parse rotation and translation solutions.
  tfm.block(0, 0, 3, 3) = result.rotation.cast<float>();
  tfm.block(0, 3, 3, 1) = result.translation.cast<float>();
  tfm(3, 3) = 1;  // Homogeneous (SE(3)).
  Eigen::Matrix4f tfm_output = fixNumerics(tfm);

  return tfm_output;
}
/*
Eigen::Matrix4f CloudOdometer::RegisterICP(Eigen::MatrixXf src,
                                          Eigen::MatrixXf tgt) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr src_pcl (new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = 0; i < src.cols(); i++) {
    pcl::PointXYZ temp;
    temp.x = src(0, i);
    temp.y = src(1, i);
    temp.z = src(2, i);
    src_pcl->points.push_back(temp);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr tgt_pcl (new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = 0; i < tgt.cols(); i++) {
    pcl::PointXYZ temp;
    temp.x = tgt(0, i);
    temp.y = tgt(1, i);
    temp.z = tgt(2, i);
    tgt_pcl->points.push_back(temp);
  }

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(src_pcl);
  icp.setInputTarget(tgt_pcl);
  icp.setMaximumIterations (params_.max_iterations);
  icp.setTransformationEpsilon (params_.trans_epsilon);
  //icp.setMaxCorrespondenceDistance (params_.max_corr_dist);
  icp.setEuclideanFitnessEpsilon (params_.eucl_fit_epsilon);
  //icp.setRANSACOutlierRejectionThreshold (params_.ransac_thresh);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);

  std::cout << "ICP Convergence: " << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;

  //Eigen::Matrix4f tfm = icp.getFinalTransformation();
  Eigen::Matrix4f tfm = icp.getFinalTransformation();
  Eigen::Matrix4f tfm_output = fixNumerics(tfm);


  std::cout << "ICP result: " << std::endl << tfm_output << std::endl;

  return tfm_output;
}
*/

}  // namespace slam
