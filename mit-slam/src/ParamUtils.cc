/**
 * @file ParamUtils.cpp
 * @brief Common useful functions for parameter handling.
 * @date Dec 30, 2020
 * @authors tonio terán (teran@mit.edu), charles oestreich (coestrei@mit.edu), jessica todd (jetodd@mit.edu)
 * @Copyright 2020 Space Systems Laboratory, MIT
 */

#include "mit_slam/ParamUtils.h"

namespace mit_slam {

BlobTracker::Params BlobParamsFromRos() {
  BlobTracker::Params params;
  ros::param::getCached("/td/mit_slam/slam_ground", params.ground);
  ros::param::getCached("/td/mit_slam/timing_verbose", params.timing_verbose);

  ros::param::getCached("/td/mit_slam/blob/max_x_distance_iss", params.max_x_distance_iss);  // [m]
  ros::param::getCached("/td/mit_slam/blob/max_y_distance_iss", params.max_y_distance_iss);  // [m]
  ros::param::getCached("/td/mit_slam/blob/max_z_distance_iss_y", params.max_z_distance_iss);  // [m]

  int test_num = 0;
  ros::param::getCached("/td/gds_test_num", test_num);
  std::string test_number_str = std::to_string(test_num);
  if (!params.ground) {
    if (test_num > 100) {
      if (test_number_str[0] == '1') {
        ros::param::getCached("/td/mit_slam/blob/max_z_distance_iss_x", params.max_z_distance_iss);
      }
      if (test_number_str[0] == '2') {
        ros::param::getCached("/td/mit_slam/blob/max_z_distance_iss_y", params.max_z_distance_iss);
      }
      if (test_number_str[0] == '3') {
        ros::param::getCached("/td/mit_slam/blob/max_z_distance_iss_z", params.max_z_distance_iss);
      }
    }
    ros::param::getCached("/td/mit_slam/blob/max_z_distance_iss_y", params.max_z_distance_iss);
  }

  ros::param::getCached("/td/mit_slam/blob/max_x_distance_ground", params.max_x_distance_ground);  // [m]
  ros::param::getCached("/td/mit_slam/blob/max_y_distance_ground", params.max_y_distance_ground);  // [m]
  ros::param::getCached("/td/mit_slam/blob/max_z_distance_ground", params.max_z_distance_ground);  // [m]

  ros::param::getCached("/td/mit_slam/blob/centroid_add_iss", params.centroid_add_iss);
  ros::param::getCached("/td/mit_slam/blob/centroid_add_ground", params.centroid_add_ground);

  ros::param::getCached("/td/mit_slam/blob/max_pcd_size", params.max_pcd_size); // [points]

  ros::param::getCached("/td/mit_slam/blob/leaf_size_x", params.leaf_size_x);
  ros::param::getCached("/td/mit_slam/blob/leaf_size_y", params.leaf_size_y);
  ros::param::getCached("/td/mit_slam/blob/leaf_size_z", params.leaf_size_z);

  return params;
}

CloudOdometer::Params CloudParamsFromRos() {
  CloudOdometer::Params params;

  ros::param::getCached("/td/mit_slam/timing_verbose", params.timing_verbose);

  // Teaser parameters.
  teaser::RobustRegistrationSolver::Params tparams;
  // Square of ratio between acceptable noise and noise bound. Usually set to 1.
  ros::param::getCached("/td/mit_slam/cloud/teaser_cbar2", tparams.cbar2);
  // Registration is much faster when not estimating scale.
  ros::param::getCached("/td/mit_slam/cloud/teaser_estimate_scaling", tparams.estimate_scaling);
  // A bound on the noise of each provided measurement.
  ros::param::getCached("/td/mit_slam/cloud/teaser_noise_bound", tparams.noise_bound);
  // Factor to multiple/divide the control parameter in the GNC algorithm.
  ros::param::getCached("/td/mit_slam/cloud/teaser_rotation_gnc_factor",
                            tparams.rotation_gnc_factor);
  // Cost threshold for the GNC rotation estimators.
  ros::param::getCached("/td/mit_slam/cloud/teaser_rotation_cost_threshold",
                            tparams.rotation_cost_threshold);
  // The algorithm to be used for estimating rotation.
  tparams.rotation_estimation_algorithm =
    teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
  // Maximum iterations allowed for the GNC rotation estimator
  tparams.rotation_max_iterations = 100;
  // Set the teaserpp parameters to CloudOdometer parameters structure.
  params.teaser_params = tparams;

  // Feature detection thresholds
  // For 2 meter scenario:
  ros::param::getCached("/td/mit_slam/cloud/feat_norm_radius", params.norm_radius);
  ros::param::getCached("/td/mit_slam/cloud/feat_fpfh_radius", params.fpfh_radius);
  ros::param::getCached("/td/mit_slam/cloud/feat_norm_scale", params.norm_scale);
  ros::param::getCached("/td/mit_slam/cloud/feat_fpfh_scale", params.fpfh_scale);
  ros::param::getCached("/td/mit_slam/cloud/feat_min_norm_radius", params.min_norm_radius);
  ros::param::getCached("/td/mit_slam/cloud/feat_min_fpfh_radius", params.min_fpfh_radius);

  // Matching parameters
  ros::param::getCached("/td/mit_slam/cloud/match_use_absolute_scale", params.use_absolute_scale);
  ros::param::getCached("/td/mit_slam/cloud/match_use_crosscheck", params.use_crosscheck);
  ros::param::getCached("/td/mit_slam/cloud/match_use_tuple_test", params.use_tuple_test);
  ros::param::getCached("/td/mit_slam/cloud/match_tuple_scale", params.tuple_scale);

  // ICP params
  ros::param::getCached("/td/mit_slam/cloud/icp_max_iterations", params.max_iterations);
  ros::param::getCached("/td/mit_slam/cloud/icp_trans_epsilon", params.trans_epsilon);
  ros::param::getCached("/td/mit_slam/cloud/icp_max_corr_dist", params.max_corr_dist);
  ros::param::getCached("/td/mit_slam/cloud/icp_eucl_fit_epsilon", params.eucl_fit_epsilon);
  ros::param::getCached("/td/mit_slam/cloud/icp_ransac_thresh", params.ransac_thresh);

  return params;
}


GraphManager::Params GraphParamsFromRos() {
  GraphManager::Params params;

  ros::param::getCached("/td/mit_slam/timing_verbose", params.timing_verbose);

  ros::param::getCached("/td/mit_slam/slam_ground", params.ground);

  int opt_iters_ros;
  ros::param::getCached("/td/mit_slam/graph/opt_iters", opt_iters_ros);
  params.opt_iters = opt_iters_ros;

  // TODO: better ways to do this for Astrobee cube (SPHERES was spherical)
  ros::param::getCached("/td/mit_slam/graph/blob_range_bias", params.blob_range_bias);


  gtsam::imuBias::ConstantBias bias0(gtsam::Vector6::Zero());
  params.bias0 = bias0;


  /// Noise models for factor graph
  // TODO: Tune these for better accuracy

  // Chaser prior noise
  // Note: GTSAM convention is rotation noise on the first 3 components, then translation
  // Bias order is accelBias, gyroBias, as in imuBias::ConstantBias.vector()
  std::vector<double> ppNM_ros;
  std::vector<double> vpNM_ros;
  std::vector<double> bpNM_ros;
  ros::param::getCached("/td/mit_slam/graph/ppNM", ppNM_ros);
  ros::param::getCached("/td/mit_slam/graph/vpNM", vpNM_ros);
  ros::param::getCached("/td/mit_slam/graph/bpNM", bpNM_ros);
  auto ppNM = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << ppNM_ros[0], ppNM_ros[1], ppNM_ros[2], ppNM_ros[3], ppNM_ros[4], ppNM_ros[5]).finished());
  auto vpNM = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(3) << vpNM_ros[0], vpNM_ros[1], vpNM_ros[2]).finished());
  auto bpNM = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << bpNM_ros[0], bpNM_ros[1], bpNM_ros[2], bpNM_ros[3], bpNM_ros[4], bpNM_ros[5]).finished());

  double blobpNM_mag;
  ros::param::getCached("/td/mit_slam/graph/blobpNM", blobpNM_mag);
  auto blobpNM = gtsam::noiseModel::Isotropic::Sigma(3, blobpNM_mag);

  // Target prior noise
  // NOTE: from Tonio: this should actually be equality, so we place a prior with
  // extremely low uncertainty. Could try to change to a PoseEquality factor
  double gpNM_mag;
  ros::param::getCached("/td/mit_slam/graph/gpNM", gpNM_mag);
  auto gpNM = gtsam::noiseModel::Isotropic::Sigma(6, gpNM_mag); // [m, rad]
  // Noise model for prior on initial translation offset from geometric frame
  double tpNM_mag;
  ros::param::getCached("/td/mit_slam/graph/tpNM", tpNM_mag);
  auto tpNM = gtsam::noiseModel::Isotropic::Sigma(3, tpNM_mag); // [m]

  // LIDAR odometry noise
  // Not sure if this is good for loop closures, but will keep them the same for now.
  std::vector<double> gNM_ros;
  std::vector<double> lcNM_ros;
  ros::param::getCached("/td/mit_slam/graph/gNM", gNM_ros);
  ros::param::getCached("/td/mit_slam/graph/lcNM", lcNM_ros);
  auto gNM = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gNM_ros[0], gNM_ros[1], gNM_ros[2], gNM_ros[3], gNM_ros[4], gNM_ros[5]).finished());
  auto lcNM = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << lcNM_ros[0], lcNM_ros[1], lcNM_ros[2], lcNM_ros[3], lcNM_ros[4], lcNM_ros[5]).finished());

  // Rotation kinematic factor noise
  double rkfNM_mag;
  ros::param::getCached("/td/mit_slam/graph/rkfNM", rkfNM_mag);
  auto rkfNM = gtsam::noiseModel::Isotropic::Sigma(3, rkfNM_mag);

  // range/bearing model for center of mass estimates
  std::vector<double> blobNM_ros;
  ros::param::getCached("/td/mit_slam/graph/blobNM", blobNM_ros);
  auto blobNM = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(3) << blobNM_ros[0], blobNM_ros[1], blobNM_ros[2]).finished());

  params.ppNM = ppNM;
  params.vpNM = vpNM;
  params.bpNM = bpNM;
  params.blobpNM = blobpNM;
  params.gpNM = gpNM;
  params.tpNM = tpNM;
  params.gNM = gNM;
  params.lcNM = lcNM;
  params.rkfNM = rkfNM;
  params.blobNM = blobNM;

  // IMU parameters (Epson G362)
  // Also referenced https://github.com/haidai/gtsam/blob/master/examples/ImuFactorsExample.cpp
  double accel_noise_sigma;
  double gyro_noise_sigma;
  double accel_bias_rw_sigma;
  double gyro_bias_rw_sigma;
  double integration_error_cov_factor;
  double bias_acc_omega_int_factor;
  ros::param::getCached("/td/mit_slam/graph/imu_accel_noise_sigma", accel_noise_sigma);
  ros::param::getCached("/td/mit_slam/graph/imu_gyro_noise_sigma", gyro_noise_sigma);
  ros::param::getCached("/td/mit_slam/graph/imu_accel_bias_rw_sigma", accel_bias_rw_sigma);   // assuming 62.5 Hz operation
  ros::param::getCached("/td/mit_slam/graph/imu_gyro_bias_rw_sigma", gyro_bias_rw_sigma);   // assuming 62.5 Hz operation
  ros::param::getCached("/td/mit_slam/graph/imu_integration_error_cov_factor", integration_error_cov_factor);
  ros::param::getCached("/td/mit_slam/graph/imu_bias_acc_omega_int_factor", bias_acc_omega_int_factor);

  gtsam::Matrix33 measured_acc_cov = gtsam::Matrix33::Identity(3,3) * pow(accel_noise_sigma, 2);
  gtsam::Matrix33 measured_omega_cov = gtsam::Matrix33::Identity(3,3) * pow(gyro_noise_sigma, 2);
  gtsam::Matrix33 integration_error_cov = gtsam::Matrix33::Identity(3,3)*integration_error_cov_factor;
  gtsam::Matrix33 bias_acc_cov = gtsam::Matrix33::Identity(3,3) * pow(accel_bias_rw_sigma, 2);
  gtsam::Matrix33 bias_omega_cov = gtsam::Matrix33::Identity(3,3) * pow(gyro_bias_rw_sigma, 2);
  gtsam::Matrix66 bias_acc_omega_int = gtsam::Matrix::Identity(6,6)*bias_acc_omega_int_factor;

  params.accel_noise_sigma = accel_noise_sigma;
  params.gyro_noise_sigma = gyro_noise_sigma;
  params.accel_bias_rw_sigma = accel_bias_rw_sigma;
  params.gyro_bias_rw_sigma = gyro_bias_rw_sigma;
  params.measured_acc_cov = measured_acc_cov;
  params.measured_omega_cov = measured_omega_cov;
  params.integration_error_cov = integration_error_cov;
  params.bias_acc_cov = bias_acc_cov;
  params.bias_omega_cov = bias_omega_cov;
  params.bias_acc_omega_int = bias_acc_omega_int;

  return params;
}


}  // namespace slam
