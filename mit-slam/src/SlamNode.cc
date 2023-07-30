/**
 * @file SLAMNode.cpp
 * @brief ROS node for SLAM.
 * @date Dec 30, 2020
 * @authors tonio terán (teran@mit.edu), charles oestreich (coestrei@mit.edu), jessica todd (jetodd@mit.edu)
 * @Copyright 2020 Space Systems Laboratory, MIT
 */

#include "mit_slam/SlamNode.h"

// Set type for state vector
typedef Eigen::Matrix<float, 1, 13> state_type;

namespace mit_slam {

/*
* Constructor
*/
SlamNode::SlamNode(ros::NodeHandle* nh, const Params &params) : params_(params) {
  // Subscribe to the chaser IMU topic.
  imu_sub_ = nh->subscribe(params_.imu_topic, 1,
                           &SlamNode::ImuMeasCallback, this);
  // Subscribe to the point cloud topic.
  pcd_sub_ = nh->subscribe(params_.pcd_topic, 1,
                           &SlamNode::PointCloudCallback, this);

  // Subscribe to ground truth states is in simulator
  if (params_.sim.compare("true") == 0) {
    chaser_gt_pose_sub_ = nh->subscribe(params_.chaser_gt_pose_topic, 1,
                                    &SlamNode::ChaserGtPoseCallback, this);
    // Subscribe to chaser ground truth twist
    chaser_gt_twist_sub_ = nh->subscribe(params_.chaser_gt_twist_topic, 1,
                                    &SlamNode::ChaserGtTwistCallback, this);
    // Subscribe to chaser ground truth pose
    target_gt_pose_sub_ = nh->subscribe(params_.target_gt_pose_topic, 1,
                                    &SlamNode::TargetGtPoseCallback, this);
    // Subscribe to chaser ground truth twist
    target_gt_twist_sub_ = nh->subscribe(params_.target_gt_twist_topic, 1,
                                    &SlamNode::TargetGtTwistCallback, this);
  }

  // Subscriber for chaser EKF
  chaser_ekf_sub_ = nh->subscribe(params_.chaser_ekf_topic, 1,
                                  &SlamNode::ChaserEKFCallback, this);

  // Publishers for state estimates
  chaser_est_pose_pub_ = nh->advertise<geometry_msgs::PoseWithCovariance>(
        params_.chaser_est_pose_topic, 1);
  target_est_pose_pub_ = nh->advertise<geometry_msgs::PoseWithCovariance>(
        params_.target_est_pose_topic, 1);
  chaser_est_twist_pub_ = nh->advertise<geometry_msgs::TwistWithCovariance>(
        params_.chaser_est_twist_topic, 1);
  target_est_twist_pub_ = nh->advertise<geometry_msgs::TwistWithCovariance>(
        params_.target_est_twist_topic, 1);

  // Publishers for other useful factor graph variables
  est_GT_pose_pub_ = nh->advertise<geometry_msgs::PoseStamped>(
      params_.est_GT_pose_topic, 1);
  est_GC_pose_pub_ = nh->advertise<geometry_msgs::PoseStamped>(
      params_.est_GC_pose_topic, 1);
  est_t_WT_pub_ = nh->advertise<geometry_msgs::PointStamped>(
      params_.est_t_WT_topic, 1);

  /// Publisher for inertia estimation
  inertia_pub_ = nh->advertise<geometry_msgs::Inertia>(
     params_.inertia_topic, 1);

  /// Publisher for timing info
  timing_info_pub_ = nh->advertise<trace_msgs::TDSlamInfo>(
      params_.timing_info_topic, 1);

  // Publishers for useful front-end measurements
  delta_pose_pub_ = nh->advertise<geometry_msgs::PoseStamped>(
      params_.delta_pose_topic, 1);
  loop_delta_pose_pub_ = nh->advertise<geometry_msgs::PoseStamped>(
      params_.loop_delta_pose_topic, 1);
  centroid_pub_ = nh->advertise<geometry_msgs::PointStamped>(
      params_.centroid_out_topic, 1);

  // Publishers for useful visualization point clouds
  if (params_.sim.compare("true") == 0 || params_.viz_hardware) {
    // Publisher for matched cloud.
    match_point_cloud_pub_ = nh->advertise<sensor_msgs::PointCloud2>(
        params_.match_point_cloud_topic, 1);
    // Publisher for estimated point cloud via delta-poses.
    est_point_cloud_pub_ = nh->advertise<sensor_msgs::PointCloud2>(
        params_.est_point_cloud_topic, 1);
        // Publisher for estimated point cloud via delta-poses.
    est_loop_point_cloud_pub_ = nh->advertise<sensor_msgs::PointCloud2>(
        params_.est_loop_point_cloud_topic, 1);
  }

  // Timer for graph updates
  graph_timer_ = nh->createTimer(ros::Duration(params_.graph_dt), &SlamNode::GraphUpdate, this);

  // Timer for inertia estimation
  inertia_est_timer_ = nh->createTimer(ros::Duration(params_.graph_dt), &SlamNode::InertiaEstimate, this);


  // Propagator for faster state publishing rates
  prop_timer_ = nh->createTimer(ros::Duration(params_.prop_dt), &SlamNode::Propagate, this);

  // Initialize frame index
  frame_idx_ = -1;

  prop_t_ = 0.0;

  // Initialize state estimates
  /*
  if (params_.ground) {
    chaser_est_state_.pos = params_.T_WC0_ground.block(0, 3, 3, 1) + params_.t_targ_ground;
    Eigen::Matrix3f R_C = params_.T_WC0_ground.block(0, 0, 3, 3);
    Eigen::Matrix3f R_T = Eigen::Matrix3f::Identity();
    chaser_est_state_.quat = mit_slam::dcm2q(R_C);
    target_est_state_.quat = mit_slam::dcm2q(R_T);
  }
  else {
    chaser_est_state_.pos = params_.T_WC0_iss.block(0, 3, 3, 1) + params_.t_targ_ISS;
    Eigen::Matrix3f R_C = params_.T_WC0_iss.block(0, 0, 3, 3);
    Eigen::Matrix3f R_T = Eigen::Matrix3f::Identity();
    chaser_est_state_.quat = mit_slam::dcm2q(R_C);
    target_est_state_.quat = mit_slam::dcm2q(R_T);
  }
  chaser_est_state_.vel = params_.v_WC0;
  chaser_est_state_.omega << 0.0, 0.0, 0.0;

  target_est_state_.pos << 0.0, 0.0, 0.0;
  target_est_state_.vel << 0.0, 0.0, 0.0;
  target_est_state_.omega << 0.0, 0.0, 0.0;
  */



  states_initialized_ = false;

  inertia_time_ = 0.0;

  // initialize gamma for angular velocity smoothing
  gamma_k_ = 1.0;
  prev_targ_omega_ = target_est_state_.omega;

  // initialize placeholders for loop closure
  T_CiCj_loop_ = Eigen::MatrixXf::Identity(4, 4);
  T_HiHj_loop_ = Eigen::MatrixXf::Identity(4, 4);

  T_CiCj_prev_ = Eigen::MatrixXf::Identity(4, 4);

  R_GT_ = Eigen::MatrixXf::Identity(3, 3);

  // Initialize loop closure counter
  loop_closure_count_ = 0;

  omega_inertia_count_ = 0;

  // Initialize point cloud available boolean
  pcds_available_ = false;

  // Initialize inertia estimation activate
  inertia_est_activate_ = false;

  /// Status booleans
  converged_ = false;
  inertia_estimated_ = false;
  dont_smooth_omega_ = false;
  if (params_.slam_spoof) {
    inertia_estimated_ = true;
  }
}

/*
* Destructor
*/
SlamNode::~SlamNode() {}

/*
* IMU data callback
*/
void SlamNode::ImuMeasCallback(const sensor_msgs::Imu &msg) {
  if (params_.activate && !params_.slam_spoof) {
      Eigen::Vector3f omega_imu;
      Eigen::Vector3f accel_imu;

      // implement biases, take out z trans and x/y rot accelerations
      if (params_.ground) {
        accel_imu << msg.linear_acceleration.x - params_.imu_bias_acc_x,
                   msg.linear_acceleration.y - params_.imu_bias_acc_y,
                   0;
        omega_imu << 0.0,
                   0.0,
                   msg.angular_velocity.z - params_.imu_bias_omega_z;
      }
      else {
        accel_imu << msg.linear_acceleration.x - params_.imu_bias_acc_x,
                   msg.linear_acceleration.y - params_.imu_bias_acc_y,
                   msg.linear_acceleration.z - params_.imu_bias_acc_z;
        omega_imu << msg.angular_velocity.x - params_.imu_bias_omega_x,
                    msg.angular_velocity.y - params_.imu_bias_omega_y,
                    msg.angular_velocity.z - params_.imu_bias_omega_z;
      }

      if (params_.use_raw_imu) {
        // Transform from IMU frame to chaser frame
        chaser_omega_ = params_.T_C2I.block(0,0,3,3) * omega_imu;
        chaser_accel_ = params_.T_C2I.block(0,0,3,3) * accel_imu;

        // IMU odometer integrates the new IMU measurement
        graph_manager_->imu_odometer_->integrateMeasurement(chaser_accel_.cast<double>(),
                                                            chaser_omega_.cast<double>(),
                                                            params_.imu_dt);
      }
  }
}

/*
* EKF callback
*/
void SlamNode::ChaserEKFCallback(const ff_msgs::EkfState::ConstPtr msg) {
  float qx = msg->pose.orientation.x;
  float qy = msg->pose.orientation.y;
  float qz = msg->pose.orientation.z;
  float qw = msg->pose.orientation.w;
  
  if (qx != 0 || qy != 0 || qz != 0 || qw != 0) {
    chaser_ekf_state_.quat << msg->pose.orientation.x,
                              msg->pose.orientation.y,
                              msg->pose.orientation.z,
                              msg->pose.orientation.w;
    chaser_ekf_state_.pos << msg->pose.position.x,
                             msg->pose.position.y,
                             msg->pose.position.z;
    chaser_ekf_state_.vel << msg->velocity.x, msg->velocity.y, msg->velocity.z;
    chaser_ekf_state_.omega << msg->omega.x, msg->omega.y, msg->omega.z;

    if (!states_initialized_ && params_.activate) {
      if (params_.ground) {
        chaser_est_state_.pos = chaser_ekf_state_.pos;
        Eigen::Matrix3f R_T = Eigen::Matrix3f::Identity();
        chaser_est_state_.quat = chaser_ekf_state_.quat;
        target_est_state_.quat = mit_slam::dcm2q(R_T);
      }
      else {
        chaser_est_state_.pos = chaser_ekf_state_.pos;
        Eigen::Matrix3f R_T = Eigen::Matrix3f::Identity();
        chaser_est_state_.quat = chaser_ekf_state_.quat;
        target_est_state_.quat = mit_slam::dcm2q(R_T);
      }
      chaser_est_state_.vel = chaser_ekf_state_.vel;
      chaser_est_state_.omega << chaser_ekf_state_.omega;

      target_est_state_.pos << 0.0, 0.0, 0.0;
      target_est_state_.vel << 0.0, 0.0, 0.0;
      target_est_state_.omega << 0.0, 0.0, 0.0;

      states_initialized_ = true;
    }

    if (!params_.use_raw_imu && params_.activate) {
      Eigen::Vector3f chaser_accel_raw;
      chaser_accel_raw << msg->accel.x, msg->accel.y, msg->accel.z;
      chaser_omega_ = chaser_ekf_state_.omega;
      chaser_accel_ = chaser_accel_raw;
      //chaser_omega_ = params_.T_C2I.block(0,0,3,3) * chaser_ekf_state_.omega;
      //chaser_accel_ = params_.T_C2I.block(0,0,3,3) * chaser_accel_raw;
      // IMU odometer integrates the new IMU measurement

      if (!params_.slam_spoof) {
        graph_manager_->imu_odometer_->integrateMeasurement(chaser_accel_.cast<double>(),
                                                            chaser_omega_.cast<double>(),
                                                            params_.ekf_dt);
      }
      else {
        // use EKF for chaser state estimates
        chaser_est_state_.vel = chaser_ekf_state_.vel;
        chaser_est_state_.omega << chaser_ekf_state_.omega;
        chaser_est_state_.quat = chaser_ekf_state_.quat;
        chaser_est_state_.pos = chaser_ekf_state_.pos;
        if (params_.ground) {
          chaser_est_state_.pos = chaser_est_state_.pos;
        }
        else {
          chaser_est_state_.pos = chaser_est_state_.pos - params_.r_RI + params_.r_TR;
        }

      }

    }
  }
}


/*
* HazCam callback
*/
void SlamNode::PointCloudCallback(const sensor_msgs::PointCloud2 &msg) {
  if (params_.activate) {
    // Save message so it can be used by graph update
    pcd_msg_ = msg;
    //std::cout << "!!!!!!!! WE GOT A HAZ CAM POINT CLOUD MESSAGE !!!!!!" << std::endl;
    // Save frame ID for visualization purposes
    params_.pcd_frame_id = msg.header.frame_id;

    // Say we now have point clouds to work with
    pcds_available_ = true;
  }
}

/*
* Graph update (via ROS timer)
*/
void SlamNode::GraphUpdate(const ros::TimerEvent& t) {
  if (params_.activate && pcds_available_ && states_initialized_ && !params_.slam_spoof) {
    std::cout << "[MIT SLAM]: NEW GRAPH UPDATE" << std::endl;
    // increment keyframe index
    frame_idx_++;

    // check if we have enough frames to do inertia estimation
    if (frame_idx_ - params_.convergence_frames > (params_.omega_meas_frames / params_.graph_dt) && !inertia_estimated_) {
      inertia_est_activate_ = true;
      std::cout << "[MIT SLAM]: ACTIVATING INERTIA ESTIMATION" << std::endl;
    }

    if (params_.verbose || params_.timing_verbose) {
      std::cout << "[MIT-SLAM] --- NEW GRAPH UPDATE ---" << std::endl;
      std::cout << std::endl;
      std::cout << "[MIT-SLAM] Frame index: " << frame_idx_ << std::endl;
    }
    std::cout << params_.verbose << std::endl;
    std::cout << params_.timing_verbose << std::endl;

    // Begin update timing measurement
    auto graph_start = std::chrono::high_resolution_clock::now();
    
    auto graph_preloop_start = std::chrono::high_resolution_clock::now();

    // Get estimate of target's centroid and truncate the point cloud
    auto blob_start = std::chrono::high_resolution_clock::now();
    Eigen::MatrixXf eigen_pcd;  // point cloud matrix in Eigen format
    std::tie(centroid_H_, eigen_pcd) = blob_tracker_->Centroid(pcd_msg_);
    auto blob_finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> blob_elapsed = blob_finish - blob_start;
    trunc_pcd_size_stat_ = eigen_pcd.cols();

    // If point cloud is empty, reset the SLAM nodelet. If not, proceed as normal.
    if (eigen_pcd.cols() == 0) {
      std::cout << "Point cloud empty, resetting SLAM nodelet." << std::endl;
      Reset();
      std::cout << "MIT SLAM nodelet successfully reset." << std::endl;
    }
    else {
      auto feat_start = std::chrono::high_resolution_clock::now();
      // Adjust point cloud feature thresholds based on centroid distance
      double new_norm_radius = cloud_odometer_->params_.norm_scale * centroid_H_.norm();
      double new_fpfh_radius = cloud_odometer_->params_.fpfh_scale * centroid_H_.norm();
      if (new_norm_radius >= cloud_odometer_->params_.min_norm_radius) {
        cloud_odometer_->params_.norm_radius = new_norm_radius;
      }
      else {
        cloud_odometer_->params_.norm_radius = cloud_odometer_->params_.min_norm_radius;
      }
      if (new_fpfh_radius >= cloud_odometer_->params_.min_fpfh_radius) {
        cloud_odometer_->params_.fpfh_radius = new_fpfh_radius;
      }
      else {
        cloud_odometer_->params_.fpfh_radius = cloud_odometer_->params_.min_fpfh_radius;
      }
      if (params_.verbose) {
        std::cout << "Norm radius: " << cloud_odometer_->params_.norm_radius << std::endl;
        std::cout << "FPFH radius: " << cloud_odometer_->params_.fpfh_radius << std::endl;
      }

      // Convert centroid to chaser frame
      Eigen::Vector4f centroid_HG_h;
      Eigen::Vector4f centroid_CG_h;
      centroid_HG_h << centroid_H_(0), centroid_H_(1), centroid_H_(2), 1;
      centroid_CG_h = params_.T_C2H * centroid_HG_h;
      Eigen::Vector3f centroid_CG = centroid_CG_h.block(0, 0, 3, 1);
      if (params_.ground) {
        centroid_CG(2) = 0.0;
      }
      if (params_.verbose) {
          std::cout << "Estimated centroid in HazCam frame: \n" << centroid_H_ << std::endl;
          std::cout << "Estimated centroid in chaser frame: \n" << centroid_CG << std::endl;
      }

      // Convert Eigen point cloud to teaser point cloud, add to databases
      teaser::PointCloud teaser_cloud;
      for (int i = 0; i < eigen_pcd.cols(); i++) {
        teaser_cloud.push_back({static_cast<float>(eigen_pcd(0,i)),
                                static_cast<float>(eigen_pcd(1,i)),
                                static_cast<float>(eigen_pcd(2,i))});
      }

      cloud_database_.push_back(teaser_cloud);
        eigen_cloud_database_.push_back(eigen_pcd);

      // Compute 3-D features, add to database
      if (!params_.use_raw_clouds) {
        teaser::FPFHCloudPtr features = cloud_odometer_->DetectFeatures(teaser_cloud);
        feature_database_.push_back(features);
      }

      auto feat_finish = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> feat_elapsed = feat_finish - feat_start;


      // If first frame, initialize graph. Else, add normal factors
      if (frame_idx_ == 0) {
        std::cout << " FRAME ID is ZERO " << std::endl;
        // Initialize G frame
        T_GC0_ << 1, 0, 0, -centroid_CG(0),
                  0, 1, 0, -centroid_CG(1),
                  0, 0, 1, -centroid_CG(2),
                  0, 0, 0,               1;

        // Temporary definition of T_GT using simulator ground truth. In future, will be estimated via conic fit optimization
        if (params_.sim.compare("true") == 0) {
          Eigen::Matrix4f T_WC_truth = Eigen::Matrix4f::Identity(4, 4);
          Eigen::Matrix3f R_WC_truth;
          Eigen::Vector3f t_WC_truth;
          R_WC_truth = mit_slam::q2dcm(chaser_gt_state_.quat(0), chaser_gt_state_.quat(1), chaser_gt_state_.quat(2), chaser_gt_state_.quat(3));
          t_WC_truth << chaser_gt_state_.pos(0), chaser_gt_state_.pos(1), chaser_gt_state_.pos(2);
          T_WC_truth.block(0, 0, 3, 3) = R_WC_truth;
          T_WC_truth.block(0, 3, 3, 1) = t_WC_truth;

          Eigen::Matrix4f T_WT_truth = Eigen::Matrix4f::Identity(4, 4);
          Eigen::Matrix3f R_WT_truth;
          Eigen::Matrix3f R_WT_truth_raw_ground;
          Eigen::Matrix3f R_180;
          R_180 << 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0;
          if (params_.ground) {
            R_WT_truth_raw_ground = mit_slam::q2dcm(target_gt_state_.quat(0), target_gt_state_.quat(1), target_gt_state_.quat(2), target_gt_state_.quat(3));
            R_WT_truth = R_WT_truth_raw_ground * R_180;
            T_WT_truth.block(0, 0, 3, 3) = R_WT_truth;
            T_GT_ = T_GC0_ * T_WC_truth.inverse() * T_WT_truth;
            R_GT_ = T_GT_.block(0, 0, 3, 3);
          }
          else {
            R_WT_truth = mit_slam::q2dcm(target_gt_state_.quat(0), target_gt_state_.quat(1), target_gt_state_.quat(2), target_gt_state_.quat(3));
            T_WT_truth.block(0, 0, 3, 3) = R_WT_truth;
            T_GT_truth_ = T_GC0_ * T_WC_truth.inverse() * T_WT_truth;
            T_GT_ = Eigen::Matrix4f::Identity(4, 4);
          }
        }
        else {
          T_GT_ = Eigen::Matrix4f::Identity(4, 4);
        }

        // Initialize chaser pose chain with the current state estimate
        Eigen::Matrix4f T_WC0 = Eigen::Matrix4f::Identity(4, 4);
        Eigen::Matrix3f R_WC0;
        Eigen::Vector3f t_WC0;
        Eigen::Vector3f v_WC0;
        R_WC0 = mit_slam::q2dcm(chaser_est_state_.quat(0), chaser_est_state_.quat(1), chaser_est_state_.quat(2), chaser_est_state_.quat(3));
        if (params_.ground) {
          t_WC0 = chaser_est_state_.pos - params_.t_targ_ground;
        }
        else {
          t_WC0 = chaser_est_state_.pos - params_.r_RI;
        }
        std::cout << "t_WC0: " << std::endl << t_WC0 << std::endl;
        T_WC0.block(0, 0, 3, 3) = R_WC0;
        T_WC0.block(0, 3, 3, 1) = t_WC0;
        v_WC0 = chaser_est_state_.vel;
        graph_manager_->InitChaserChain(T_WC0, v_WC0, centroid_CG);

        // Initialize geometric frame (target) pose chain with the current state estimate
        graph_manager_->InitGeomChain(T_GC0_, T_WC0);
      }
      else {
        auto match_start = std::chrono::high_resolution_clock::now();
        if (!params_.use_raw_clouds) {
          // Match features between subsequent frames
          std::vector<std::pair<int,int>> matches = cloud_odometer_->MatchFeatures(cloud_database_[frame_idx_ - 1],
                                                                                   cloud_database_[frame_idx_],
                                                                                   feature_database_[frame_idx_ - 1],
                                                                                   feature_database_[frame_idx_]);
          // Randomly down-sample the matches if there are too many
          if (matches.size() > params_.downsample_thresh) {
            matches_final_ = DownSampleMatches(matches);
          }
          else {
            matches_final_ = matches;
          }
          if (params_.verbose || params_.timing_verbose) {
            std::cout << "[MIT-SLAM] Number of matches (before downsampling): " << matches.size() << std::endl;
            std::cout << "[MIT-SLAM] Number of matches (after downsampling): " << matches_final_.size() << std::endl;
          }
          num_matches_stat_ = matches.size();
        }
        else {
          int match_list_size = 0;
          if (cloud_database_[frame_idx_ -1].size() <= cloud_database_[frame_idx_].size()) {
            match_list_size = cloud_database_[frame_idx_ -1].size();
          }
          else {
            match_list_size = cloud_database_[frame_idx_].size();
          }
          std::vector<std::pair<int, int>> matches;
          for (int i = 0; i < match_list_size; i++) {
            for (int j = 0; j < match_list_size; j++) {
              std::pair<int, int> match = std::pair<int, int>(i, j);
              matches.push_back(match);
            }
          }
          matches_final_ = matches;
          if (params_.verbose || params_.timing_verbose) {
            std::cout << "[MIT-SLAM] Number of matches (before downsampling): " << matches.size() << std::endl;
            std::cout << "[MIT-SLAM] Number of matches (after downsampling): " << matches_final_.size() << std::endl;
          }
          num_matches_stat_ = matches.size();

        }

        auto match_finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> match_elapsed = match_finish - match_start;


        auto reg_start = std::chrono::high_resolution_clock::now();

        T_HiHj_ = cloud_odometer_->Register(cloud_database_[frame_idx_ - 1],
                                                           cloud_database_[frame_idx_],
                                                           matches_final_);
        //T_HiHj_ = cloud_odometer_->RegisterICP(eigen_cloud_database_[frame_idx_ - 1],
                                               //eigen_cloud_database_[frame_idx_]);


        T_CiCj_ = (params_.T_C2H * T_HiHj_ * params_.T_C2H.inverse()).inverse();
        if (params_.verbose || params_.timing_verbose) {
          std::cout << "[MIT-SLAM] T_GC odometry: \n" << T_CiCj_ << std::endl;
        }

        // Check for bad target pose odometry measurements
        CheckBadOdom();

        auto reg_finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> reg_elapsed = reg_finish - reg_start;


        // Attempt loop closure
        auto graph_preloop_finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> graph_preloop_elapsed = graph_preloop_finish - graph_preloop_start;

        auto loop_start = std::chrono::high_resolution_clock::now();
        if ((frame_idx_ >= 6) && params_.loop_closure_enable && (std::chrono::duration<double>(graph_preloop_elapsed).count() < 1.25)) {
          AttemptLoopClosure();
        }
        auto loop_finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> loop_elapsed = loop_finish - loop_start;

        // Add factors to graph and solve for new state estimates via iSAM2
        auto isam_start = std::chrono::high_resolution_clock::now();
        graph_manager_->AddFactors(T_CiCj_, frame_idx_,
                                   T_CiCj_loop_, loop_idx_, loop_closure_success_,
                                   centroid_CG);

        // Get estimates from graph
        gtsam::Symbol T('T', frame_idx_), G('G', frame_idx_), v('v', frame_idx_), t('t', 0), l('l', 0);
        gtsam::Symbol Tprev('T', frame_idx_ - 1), Gprev('G', frame_idx_ - 1);
        Eigen::Matrix4f T_WCi = graph_manager_->estimate_.at<gtsam::Pose3>(T).matrix().cast<float>(); // Chaser pose w/r to world
        Eigen::Vector3f v_WCi = graph_manager_->estimate_.at<gtsam::Velocity3>(v).cast<float>(); // Chaser velocity w/r to world
        Eigen::Matrix4f T_GCi = graph_manager_->estimate_.at<gtsam::Pose3>(G).matrix().cast<float>(); // Chaser pose w/r to geometric
        Eigen::Vector3f t_GT = graph_manager_->estimate_.at<gtsam::Point3>(t).matrix().cast<float>(); // Estimated t between G/T frames
        Eigen::Vector3f t_WT = graph_manager_->estimate_.at<gtsam::Point3>(l).matrix().cast<float>(); // Estimated t between W/T frames (modeling error)
        Eigen::Matrix4f T_WCprev = graph_manager_->estimate_.at<gtsam::Pose3>(Tprev).matrix().cast<float>(); // Previous chaser pose wrt world
        Eigen::Matrix4f T_GCprev = graph_manager_->estimate_.at<gtsam::Pose3>(Gprev).matrix().cast<float>(); // Previous chase pose wrt world

        if (!params_.ground) {
          T_GT_.block(0, 0, 3, 3) = R_GT_;
        }
        T_GT_.block(0, 3, 3, 1) = t_GT;

        // Get target pose estimate in world frame
        Eigen::Matrix4f T_WGi = T_WCi * T_GCi.inverse();
        Eigen::Matrix4f T_WTi = T_WGi * T_GT_;
        T_WTi.block(0, 3, 3, 1) = Eigen::MatrixXf::Zero(3,1);

        // Compute angular velocities
        Eigen::Matrix4f T_WGprev = T_WCprev * T_GCprev.inverse(); // Previous geometric pose wrt world
        Eigen::Matrix4f T_WTprev = T_WGprev * T_GT_;
        Eigen::Matrix3f R_WTi = T_WTi.block(0, 0, 3, 3); // Rotation matrix for current geometric pose wrt world
        Eigen::Matrix3f R_WGi = T_WGi.block(0, 0, 3, 3); // Rotation matrix for current geometric pose wrt world
        Eigen::Matrix3f R_WTprev = T_WTprev.block(0, 0, 3, 3); // Rotation matrix for previous geometric pose wrt world
        Eigen::Matrix3f R_WGprev = T_WGprev.block(0, 0, 3, 3); // Rotation matrix for previous geometric pose wrt world
        Eigen::Vector3f omega_G_ij = ComputeAngularVelocity(R_WGi, R_WGprev, params_.graph_dt);
        for (int i = 0; i < 3; i++) {
          if (std::isnan(omega_G_ij(i))) {
              omega_G_ij(i) = 0.0;
          }
        }

        Eigen::Matrix3f R_WCi = T_WCi.block(0, 0, 3, 3); // Rotation matrix for current chaser pose wrt world
        Eigen::Matrix3f R_WCprev = T_WCprev.block(0, 0, 3, 3); // Rotation matrix for previous chaser pose wrt world
        Eigen::Vector3f omega_C_ij = ComputeAngularVelocity(R_WCi, R_WCprev, params_.graph_dt);
        for (int i = 0; i < 3; i++) {
          if (std::isnan(omega_C_ij(i))) {
              omega_C_ij(i) = 0.0;
          }
        }

        // Smooth target angular velocity
        Eigen::Vector3f omega_G_ij_smooth;
        gamma_k_1_ = exp(params_.ff-1) * (1-exp((params_.ff-1)*(frame_idx_+1))) / (1 - exp(params_.ff-1));
        gamma_k_ = gamma_k_1_;
        if (frame_idx_ > (params_.convergence_frames / params_.graph_dt)) {
          if (!dont_smooth_omega_) {
            omega_G_ij_smooth = exp(params_.ff-1) / gamma_k_1_ * (gamma_k_ * prev_targ_omega_ + omega_G_ij);
          }
          else {
            omega_G_ij_smooth = omega_G_ij;
            dont_smooth_omega_ = true;
          }
          //omega_G_ij_smooth = exp(params_.ff-1) / gamma_k_1_ * (gamma_k_ * prev_targ_omega_ + omega_G_ij);
          //omega_G_ij_smooth = omega_G_ij;
          prev_targ_omega_ = omega_G_ij_smooth;
          // Add target angular velocity to measurement database if past convergence
          //omega_G_ij_smooth = omega_G_ij;
          omega_inertia_meas_.push_back(omega_G_ij_smooth);
          omega_inertia_count_++;
          converged_ = true;
        }
        else {
          omega_G_ij_smooth = omega_G_ij;
        }
        prev_targ_omega_ = omega_G_ij_smooth;

        // Get forms used in ROS publishing
        Eigen::Vector3f v_WTi = Eigen::MatrixXf::Zero(3,1); // Zero target v
        Eigen::Vector4f quat_WCi = mit_slam::dcm2q(T_WCi.block(0, 0, 3, 3));
        Eigen::Vector4f quat_WTi = mit_slam::dcm2q(T_WTi.block(0, 0, 3, 3));
        Eigen::Vector3f pos_WCi = T_WCi.block(0, 3, 3, 1);
        Eigen::Vector3f pos_WTi = T_WTi.block(0, 3, 3, 1);

        // Zero out necessary values for ground scenario
        if (params_.ground) {
          quat_WCi(0) = 0.0;
          quat_WCi(1) = 0.0;
          quat_WCi.normalize();
          pos_WCi(2) = 0.0;

          quat_WTi(0) = 0.0;
          quat_WTi(1) = 0.0;
          quat_WTi.normalize();
          pos_WTi(2) = 0.0;

          v_WCi(2) = 0.0;
          omega_C_ij(0) = 0.0;
          omega_C_ij(1) = 0.0;

          omega_G_ij_smooth(0) = 0.0;
          omega_G_ij_smooth(1) = 0.0;
        }

        // Formulate state structs
        if (params_.ground) {
          chaser_est_state_.pos = pos_WCi + params_.t_targ_ground;
          target_est_state_.pos = pos_WTi + params_.t_targ_ground;
        }
        else {
          //chaser_est_state_.pos = pos_WCi + params_.r_RI - params_.r_TR;
          //target_est_state_.pos = pos_WTi + params_.r_RI - params_.r_TR;
          chaser_est_state_.pos = pos_WCi + params_.r_TR;
          target_est_state_.pos = pos_WTi + params_.r_TR;
        }
        chaser_est_state_.vel = v_WCi;
        chaser_est_state_.quat = quat_WCi;
        chaser_est_state_.omega = omega_C_ij;

        target_est_state_.vel = v_WTi;
        target_est_state_.quat = quat_WTi;
        target_est_state_.omega = R_GT_.transpose() * omega_G_ij_smooth;

        auto isam_finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> isam_elapsed = isam_finish - isam_start;

        // Finish timing for graph update
        auto graph_finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> graph_elapsed = graph_finish - graph_start;
        if (params_.verbose || params_.timing_verbose) {
            std::cout << "Loop closure count: " << loop_closure_count_ << std::endl;
            std::cout << "Update elapsed time: " << graph_elapsed.count() << std::endl;
            std::cout << "\n[MIT-SLAM] --- END OF GRAPH UPDATE ---\n\n";
        }

        std::cout << "\n[PUBLISHING] \n\n";
        // Publish state estimates. In future, this will happen in propagate function
        PublishChaserPoseEst(chaser_est_state_.quat, chaser_est_state_.pos);
        PublishTargetPoseEst(target_est_state_.quat, target_est_state_.pos);
        PublishChaserTwistEst(chaser_est_state_.vel, chaser_est_state_.omega);
        PublishTargetTwistEst(target_est_state_.vel, target_est_state_.omega);

        // Publish other useful variables from factor graph
        //Eigen::Matrix4f T_GT_est = Eigen::Matrix4f::Identity(4, 4);
        //T_GT_est.block(0, 3, 3, 1) = t_GT;  // Will eventually include R_GT once we have principal axes
        PublishGTPoseEst(T_GT_, params_.pcd_frame_id);
        PublishGCPoseEst(T_GCi, params_.pcd_frame_id);
        PublishtWTEst(t_WT, params_.pcd_frame_id);

        // Publish useful front-end measurements
        PublishDeltaPose(T_CiCj_, params_.pcd_frame_id);
        PublishLoopDeltaPose(T_CiCj_loop_, params_.pcd_frame_id);
        PublishCentroid(centroid_H_, params_.pcd_frame_id);

        // Publish timing info
        trace_msgs::TDSlamInfo timing_info_msg;
        timing_info_msg.stamp = ros::Time::now();
        timing_info_msg.converged = converged_;
        timing_info_msg.inertia_estimated = inertia_estimated_;
        timing_info_msg.graph_update_time = std::chrono::duration<double>(graph_elapsed).count();
        timing_info_msg.blob_time = std::chrono::duration<double>(blob_elapsed).count();
        timing_info_msg.feat_time = std::chrono::duration<double>(feat_elapsed).count();
        timing_info_msg.match_time = std::chrono::duration<double>(match_elapsed).count();
        timing_info_msg.reg_time = std::chrono::duration<double>(reg_elapsed).count();
        timing_info_msg.loop_time = std::chrono::duration<double>(loop_elapsed).count();
        timing_info_msg.isam_time = std::chrono::duration<double>(isam_elapsed).count();
        timing_info_msg.trunc_pcd_size = trunc_pcd_size_stat_;
        timing_info_msg.num_matches = num_matches_stat_;
        timing_info_msg.inertia_time = inertia_time_;
        timing_info_msg.loop_closure_count = loop_closure_count_;
        timing_info_pub_.publish(timing_info_msg);

        // If in sim, publish point clouds for visualization
        if (params_.sim.compare("true") == 0 || params_.viz_hardware) {
          PublishMatchPointCloud(eigen_pcd, matches_final_, params_.pcd_frame_id);
          // PublishEstPointCloud(T_HiHj_, params_.pcd_frame_id);
          if (loop_closure_success_) {
              prev_loop_eigen_pcd_ = eigen_cloud_database_[loop_idx_];
              PublishEstLoopPointCloud(T_HiHj_loop_, params_.pcd_frame_id);
          }
          prev_eigen_pcd_ = eigen_pcd;
        }

        // If visualizing hardware data, publish necessary transforms
        if (params_.viz_hardware) {
          PublishChaserTransforms();
        }

        // Print estimate outputs to console if desired
        if (params_.verbose) {
          ConsoleOutput();
        }
      }
      // Reset loop closure variables
      T_CiCj_loop_ = Eigen::MatrixXf::Identity(4, 4);
      T_HiHj_loop_ = Eigen::MatrixXf::Identity(4, 4);
      loop_closure_success_ = false;
      std::cout << "\n [MIT SLAM]: ------------- RESET COMPLETE --------------- \n\n";
    }
  }
}


/*
* Inertia estimation (via ROS timer when activated)
*/
void SlamNode::InertiaEstimate(const ros::TimerEvent& t) {
  if (inertia_est_activate_ && !params_.slam_spoof) {
    // Estimate inertial properties

    if (!params_.ground && params_.compute_inertia) {

      auto inertia_start = std::chrono::high_resolution_clock::now();

      Eigen::Matrix<double, 3, Eigen::Dynamic> Gn_omega;
      Gn_omega.resize(3, omega_inertia_count_);
      for (int i = 0; i < omega_inertia_count_; i++) {
        Gn_omega.col(i) = omega_inertia_meas_[i].cast<double>();
      }

      // Initial conditions.
      const Eigen::Vector3d theta_BG_init(0, 0, 0);
      Eigen::Vector3d theta_BG_est = theta_BG_init;
      Eigen::Quaterniond q_BG_est{Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())};

      // Now try to optimize.
      auto* cost_function = pao::PrincipalAxesAlignmentCost::Create(Gn_omega);
      ceres::Problem problem;
      problem.AddResidualBlock(cost_function, nullptr, q_BG_est.coeffs().data());
      problem.SetParameterization(q_BG_est.coeffs().data(),
                                  new ceres::EigenQuaternionParameterization);
      ceres::Solver::Options options;
      options.linear_solver_type = ceres::DENSE_QR;
      options.minimizer_progress_to_stdout = true;
      ceres::Solver::Summary summary;
      ceres::Solve(options, &problem, &summary);
      std::cout << summary.BriefReport() << "\n";

      const Eigen::AngleAxisd aa_BG_est(q_BG_est);
      std::cout << "q  est: " << q_BG_est.coeffs().transpose() << std::endl;
      std::cout << "aa est: " << aa_BG_est.angle() * aa_BG_est.axis().transpose()
                << "\n";

      Eigen::Vector4f q_GP;
      q_GP << (float)q_BG_est.x(), (float)q_BG_est.y(), (float)q_BG_est.z(), (float)q_BG_est.w();
      Eigen::Matrix3f R_GP = mit_slam::q2dcm(q_GP(0), q_GP(1), q_GP(2), q_GP(3));

      /*Eigen::Matrix3f R_x;
      R_x << 1, 0, 0,
              0, 0, -1,
              0, 1, 0;
      Eigen::Matrix3f R_y;
      R_y << 0, 0, 1,
              0, 1, 0,
              -1, 0, 0;
              */
      Eigen::Matrix3f R_GE;
      sph::PrincipalAxesOpt<double> principal_axes_object(Gn_omega.cast<double>());
      R_GE = principal_axes_object.FindBodyFrame(R_GP.cast<double>().transpose()).cast<float>();
      R_GT_ = R_GE * params_.R_TP.transpose();




      std::cout << "---------------------------------" << std::endl;
      std::cout << "---------------------------------" << std::endl;
      std::cout << " PRINCIPAL AXES ESTIMATED               " << std::endl;
      std::cout << "---------------------------------" << std::endl;
      std::cout << "---------------------------------" << std::endl;
      std::cout << "---------------------------------" << std::endl;
      std::cout << "---------------------------------" << std::endl;
      std::cout << "---------------------------------" << std::endl;
      std::cout << "---------------------------------" << std::endl;

      auto inertia_finish = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> inertia_elapsed = inertia_finish - inertia_start;
      inertia_time_ = std::chrono::duration<double>(inertia_elapsed).count();

    }

    Eigen::Matrix3f J_est;
    // Nominal Envisat
    J_est << 17023.3, 397.1, -2171.4, 397.1, 124825.7, 344.2, -2171.4, 344.2, 129112.2; // kg-m^2

    PublishInertia(J_est);

    inertia_est_activate_ = false;
    inertia_estimated_ = true;
    dont_smooth_omega_ = true;
  }
}


/*
* Create polhode for inertia estimation unit test
*/
std::tuple<Eigen::VectorXd, Eigen::MatrixXd> SlamNode::CreateCleanPolhode(size_t N) {
  // Create rigid body to generate data for testing.
  pao::RigidBodyParams params;
  params.J = Eigen::Vector3d{1.239, 1.1905, 1.0}.asDiagonal();  // PAOERES
  params.R_WB0 = Eigen::Matrix3d::Identity();   // Initial orientation.
  params.init = pao::DynamicsInitType::OMEGA0;  // Initialize with omega0;
  params.omega0 = Eigen::Vector3d{0, 0.939392242898362, 0.500486277097766};
  pao::RigidBodyRotation rbr{params};  // HE case.

  Eigen::VectorXd times = Eigen::VectorXd::LinSpaced(N, 0.0, 4.0 * rbr.T());
  return std::tuple<Eigen::VectorXd, Eigen::MatrixXd> {times, rbr.PredictOmega(times)};
  //return {times, rbr.PredictOmega(times)};
}

/*
* Write data for inertia estimation unit test
*/
void SlamNode::WriteDataToDisk(size_t N, const Eigen::VectorXd& times,
                     const Eigen::MatrixXd& Bc_omega_tru,
                     const Eigen::MatrixXd& Bn_omega_tru,
                     const Eigen::MatrixXd& Gn_omega, const double rads,
                     const Eigen::Vector3d& axis,
                     const Eigen::AngleAxisd& aa_BG_est) {
  const size_t cols = 1 + 3 + 3 + 3;
  Eigen::MatrixXd data = Eigen::MatrixXd::Zero(N, cols);
  data.block(0, 0, N, 1) = times;
  data.block(0, 1, N, 3) = Bc_omega_tru.transpose();
  data.block(0, 4, N, 3) = Bn_omega_tru.transpose();
  data.block(0, 7, N, 3) = Gn_omega.transpose();

  std::string path = "/home/charles/td_ws/polhode/test_results/unittest.dat";
  std::ofstream ostream(path);
  ostream << data << std::endl;
  ostream.close();

  path = "/home/charles/td_ws/polhode/test_results/out.tru";
  ostream = std::ofstream(path);
  ostream << rads << " " << axis.transpose() << " "
          << aa_BG_est.angle() * aa_BG_est.axis().transpose() << std::endl;
  ostream.close();
}

/*
*  Reset SLAM node (when target point cloud is empty)
*/
void SlamNode::Reset() {
  // Re-initialize variables
  frame_idx_ = -1;
  gamma_k_ = 1.0;
  prev_targ_omega_ = target_est_state_.omega;
  T_CiCj_loop_ = Eigen::MatrixXf::Identity(4, 4);
  T_HiHj_loop_ = Eigen::MatrixXf::Identity(4, 4);
  T_CiCj_prev_ = Eigen::MatrixXf::Identity(4, 4);
  loop_closure_count_ = 0;
  omega_inertia_count_ = 0;
  pcds_available_ = false;
  converged_ = false;
  inertia_estimated_ = false;
  states_initialized_ = false;
  prop_t_ = 0.0;

  // Clear databases
  feature_database_.clear();
  eigen_cloud_database_.clear();
  cloud_database_.clear();

  // Restart blobtracker, cloudodometer, and graphmanager
  mit_slam::BlobTracker::Params blob_params = mit_slam::BlobParamsFromRos();
  SetupBlobTracker(blob_params);
  mit_slam::CloudOdometer::Params cloud_params = mit_slam::CloudParamsFromRos();
  SetupCloudOdometer(cloud_params);
  mit_slam::GraphManager::Params graph_params = mit_slam::GraphParamsFromRos();
  SetupGraphManager(graph_params);
}

/*
* Randomly down-sample matches to speed up point cloud registration
*/
std::vector<std::pair<int,int>> SlamNode::DownSampleMatches(const std::vector<std::pair<int,int>> correspondences) {
  std::random_device rd;
  std::mt19937 prng(rd());
  std::vector<std::pair<int,int>> downsampled_matches;
  std::vector<int> match_idx(correspondences.size());
  for (int i = 0; i < match_idx.size(); i++) {
    match_idx[i] = i;
  }
  std::shuffle(match_idx.begin(), match_idx.end(), prng);
  for (int j = 0; j < params_.downsample_thresh; j++) {
    downsampled_matches.push_back(correspondences[match_idx[j]]);
  }
  return downsampled_matches;
}

/*
* Check for unreasonable odometry estimates, set to previous odometry if so
*/
void SlamNode::CheckBadOdom() {
  float odom_angle;
  Eigen::Vector4f quat_odom = dcm2q(T_CiCj_.block(0, 0, 3, 3));
  Eigen::Vector3f quat_odom_3d;
  quat_odom_3d << quat_odom(0), quat_odom(1), quat_odom(2);
  odom_angle = 2.0 * asin(quat_odom_3d.norm());

  // check if translation and rotation are way too large for reasonable odometry
  if (std::abs(T_CiCj_(0,3)) > params_.cloud_odom_bad_x ||
      std::abs(T_CiCj_(1,3)) > params_.cloud_odom_bad_y ||
      std::abs(T_CiCj_(2,3)) > params_.cloud_odom_bad_z ||
      std::abs(odom_angle) > params_.cloud_odom_bad_alpha) {
    if (frame_idx_ == 1) {
       std::cout << "[MIT-SLAM] Bad estimate, setting to zero odometry." << std::endl;
       T_CiCj_ = Eigen::MatrixXf::Identity(4, 4);
    }
    else {
      std::cout << "[MIT-SLAM] Bad estimate, setting to previous odometry." << std::endl;
      T_CiCj_ = T_CiCj_prev_;
    }
  }
  else {
    T_CiCj_prev_ = T_CiCj_;
  }
}

/*
*  Attempt to add loop closure to factor graph
*/
void SlamNode::AttemptLoopClosure() {
  std::random_device rd;
  std::mt19937 prng(rd());

  // Define range of past frames to choose from
  int hist_window_start = 0;
  if (frame_idx_ >= params_.loop_idx_hist) {
    hist_window_start = frame_idx_ - params_.loop_idx_hist;
  }
  std::uniform_int_distribution<int> dis(hist_window_start, frame_idx_ - 2);
  // randomly pick a past frame
  loop_idx_ = dis(prng);

  // Match features
  std::vector<std::pair<int,int>> matches = cloud_odometer_->MatchFeatures(cloud_database_[loop_idx_],
                                                                           cloud_database_[frame_idx_],
                                                                           feature_database_[loop_idx_],
                                                                           feature_database_[frame_idx_]);
  // Only proceed with the loop closure if there are a lot of matches
  if (matches.size() > params_.loop_match_thresh) {
    // Randomly down-sample the matches if there are too many
    if (matches.size() > params_.downsample_thresh) {
      matches_final_loop_ = DownSampleMatches(matches);
    }
    else {
      matches_final_loop_ = matches;
    }

    // Register the matches
    T_HiHj_loop_ = cloud_odometer_->Register(cloud_database_[loop_idx_],
                                             cloud_database_[frame_idx_],
                                             matches_final_loop_);

   //T_HiHj_loop_ = cloud_odometer_->RegisterICP(eigen_cloud_database_[loop_idx_],
                                      //    eigen_cloud_database_[frame_idx_]);
    // Transform to chaser body frame
    T_CiCj_loop_ = (params_.T_C2H * T_HiHj_loop_ * params_.T_C2H.inverse()).inverse();

    // Check to make sure loop closure odometry is less than a certain rotation/translation
    // This avoids symmetry issues
    Eigen::Vector3f t_CiCj_loop = T_CiCj_loop_.block(0, 3, 3, 1);
    Eigen::Vector4f q_CiCj_loop = mit_slam::dcm2q(T_CiCj_loop_.block(0, 0, 3, 3));
    Eigen::Vector3f q_3d_CiCj_loop;
    q_3d_CiCj_loop << q_CiCj_loop(0), q_CiCj_loop(1), q_CiCj_loop(2);
    float angle = 2.0 * asin(q_3d_CiCj_loop.norm());
    if (angle < params_.loop_max_rot && t_CiCj_loop.norm() < params_.loop_max_trans) {
      loop_closure_count_++;
      loop_closure_success_ = true;
      if (params_.verbose) {
        std::cout << "SUCCESSFUL LOOP CLOSURE: Frames " << loop_idx_ << " and " << frame_idx_ << std::endl;
        std::cout << "Loop closure angle: " << angle << std::endl;
        std::cout << "Loop closure delta pose: \n" << T_CiCj_loop_ << std::endl;
      }
    }
    else {
      loop_closure_success_ = false;
      T_CiCj_loop_ = Eigen::MatrixXf::Identity(4, 4);
      T_HiHj_loop_ = Eigen::MatrixXf::Identity(4, 4);

    }
  }
  else {
    loop_closure_success_ = false;
    T_CiCj_loop_ = Eigen::MatrixXf::Identity(4, 4);
    T_HiHj_loop_ = Eigen::MatrixXf::Identity(4, 4);
  }
}

/*
*  Propagation function for faster state updates. Currently being improved, not used
*/
void SlamNode::Propagate(const ros::TimerEvent& t) {

  if (params_.slam_spoof) {
    inertia_estimated_ = true;
  }

  if (params_.activate && states_initialized_ && (params_.slam_state_mode or params_.slam_spoof) && inertia_estimated_) {

    if (prop_t_ == 0.0 && params_.slam_spoof) {
      // target off of initial state parameters
      std::string test_number_str = std::to_string(params_.test_number);
      target_est_state_.quat << 0.0, 0.0, 0.0, 1.0;
      target_est_state_.pos << 0.0, 0.0, 0.0;
      if (!params_.ground) {
        target_est_state_.pos = target_est_state_.pos + params_.r_TR;
      }
      target_est_state_.vel << 0.0, 0.0, 0.0;
      if (params_.test_number > 99) {
        if (test_number_str[1] == '1') {
          target_est_state_.omega << 0, 0.061707, 0.061707;
        }
        else if (test_number_str[1] == '2') {
          target_est_state_.omega << 0.0873, 0.0017, 0.0;
          target_est_state_.quat << 0.0, 0.0, -0.7071, 0.7071;
        }
        else {
          target_est_state_.omega << 0.0, 0.0, 0.0;
        }
      }
      else {
        target_est_state_.omega << 0, 0.061707, 0.061707;
      }
    }

    if (prop_t_ > 0.0) {

      if (params_.ground) {
          // Initialize with latest state (whether from propagate or point cloud callback)
          state_type current_chaser_state;
          state_type current_target_state;

          chaser_est_state_.quat(0) = 0.0;
          chaser_est_state_.quat(1) = 0.0;
          chaser_est_state_.quat.normalize();
          chaser_est_state_.pos(2) = -0.7;

          target_est_state_.quat(0) = 0.0;
          target_est_state_.quat(1) = 0.0;
          target_est_state_.quat.normalize();
          target_est_state_.pos(1) = -0.5;
          target_est_state_.pos(2) = -0.7;

          chaser_est_state_.vel(2) = 0.0;
          chaser_est_state_.omega(0) = 0.0;
          chaser_est_state_.omega(1) = 0.0;

          target_est_state_.vel(2) = 0.0;
          target_est_state_.omega(0) = 0.0;
          target_est_state_.omega(1) = 0.0;

          current_chaser_state.segment(0,3) = chaser_est_state_.pos;
          current_chaser_state.segment(3,3) = chaser_est_state_.vel;
          current_chaser_state.segment(6,4) = chaser_est_state_.quat;
          current_chaser_state.segment(10,3) = chaser_est_state_.omega;

          current_target_state.segment(0,3) = target_est_state_.pos;
          current_target_state.segment(3,3) = target_est_state_.vel;
          current_target_state.segment(6,4) = target_est_state_.quat;
          current_target_state.segment(10,3) = target_est_state_.omega;

          // Integrate over propagation time span
          double t_next = prop_t_ + params_.prop_dt;
          if (params_.ground) {
            if (!params_.slam_spoof) {
              boost::numeric::odeint::integrate_adaptive(chaser_stepper_, dynamic_step_ground_{params_.J_chaser}, current_chaser_state, prop_t_, t_next, params_.prop_dt);
            }
            boost::numeric::odeint::integrate_adaptive(target_stepper_, dynamic_step_ground_{params_.J_target}, current_target_state, prop_t_, t_next, params_.prop_dt);
          }
          else {
            if (!params_.slam_spoof) {
              boost::numeric::odeint::integrate_adaptive(chaser_stepper_, dynamic_step_{params_.J_chaser}, current_chaser_state, prop_t_, t_next, params_.prop_dt);
            }
            boost::numeric::odeint::integrate_adaptive(target_stepper_, dynamic_step_{params_.J_target}, current_target_state, prop_t_, t_next, params_.prop_dt);
          }

          // Assign result to the next current state
          chaser_est_state_.pos = current_chaser_state.segment(0,3);
          chaser_est_state_.vel = current_chaser_state.segment(3,3);
          chaser_est_state_.quat = current_chaser_state.segment(6,4);
          chaser_est_state_.omega = current_chaser_state.segment(10,3);

          target_est_state_.pos = current_target_state.segment(0,3);
          target_est_state_.vel = current_target_state.segment(3,3);
          target_est_state_.quat = current_target_state.segment(6,4);
          target_est_state_.omega = current_target_state.segment(10,3);

          // Publish result
            PublishChaserPoseEst(chaser_est_state_.quat, chaser_est_state_.pos);
            PublishTargetPoseEst(target_est_state_.quat, target_est_state_.pos);
            PublishChaserTwistEst(chaser_est_state_.vel, chaser_est_state_.omega);
            PublishTargetTwistEst(target_est_state_.vel, target_est_state_.omega);
      }
      else {
        state_type current_chaser_state;
        state_type current_target_state;

        current_chaser_state.segment(0,3) = chaser_est_state_.pos;
        current_chaser_state.segment(3,3) = chaser_est_state_.vel;
        current_chaser_state.segment(6,4) = chaser_est_state_.quat;
        current_chaser_state.segment(10,3) = chaser_est_state_.omega;

        current_target_state.segment(0,3) = target_est_state_.pos;
        current_target_state.segment(3,3) = target_est_state_.vel;
        current_target_state.segment(6,4) = target_est_state_.quat;
        current_target_state.segment(10,3) = target_est_state_.omega;


        // Integrate over propagation time span
        double t_next = prop_t_ + params_.prop_dt;

        if (params_.ground) {
          if (!params_.slam_spoof) {
            boost::numeric::odeint::integrate_adaptive(chaser_stepper_, dynamic_step_ground_{params_.J_chaser}, current_chaser_state, prop_t_, t_next, params_.prop_dt);
          }
          boost::numeric::odeint::integrate_adaptive(target_stepper_, dynamic_step_ground_{params_.J_target}, current_target_state, prop_t_, t_next, params_.prop_dt);
        }
        else {
          if (!params_.slam_spoof) {
            boost::numeric::odeint::integrate_adaptive(chaser_stepper_, dynamic_step_{params_.J_chaser}, current_chaser_state, prop_t_, t_next, params_.prop_dt);
          }
          boost::numeric::odeint::integrate_adaptive(target_stepper_, dynamic_step_{params_.J_target}, current_target_state, prop_t_, t_next, params_.prop_dt);
        }

        // Assign result to the next current state
        chaser_est_state_.pos = current_chaser_state.segment(0,3);
        chaser_est_state_.vel = current_chaser_state.segment(3,3);
        chaser_est_state_.quat = current_chaser_state.segment(6,4);
        chaser_est_state_.omega = current_chaser_state.segment(10,3);

        target_est_state_.pos = current_target_state.segment(0,3);
        target_est_state_.vel = current_target_state.segment(3,3);
        target_est_state_.quat = current_target_state.segment(6,4);
        target_est_state_.omega = current_target_state.segment(10,3);

        // Publish result
          PublishChaserPoseEst(chaser_est_state_.quat, chaser_est_state_.pos);
          PublishTargetPoseEst(target_est_state_.quat, target_est_state_.pos);
          PublishChaserTwistEst(chaser_est_state_.vel, chaser_est_state_.omega);
          PublishTargetTwistEst(target_est_state_.vel, target_est_state_.omega);
      }

    }
    prop_t_ += params_.prop_dt;
  }
}

void SlamNode::ConsoleOutput() {
    if (params_.sim.compare("true") == 0) {
      std::cout << "[MIT-SLAM] Truth chaser position in world frame: \n" << chaser_gt_state_.pos(0)
                                                                         << " " << chaser_gt_state_.pos(1)
                                                                         << " " << chaser_gt_state_.pos(2)  <<  std::endl;
    }
    else {
      std::cout << "[MIT-SLAM] Truth chaser position in world frame: \n" << chaser_ekf_state_.pos(0)
                                                                         << " " << chaser_ekf_state_.pos(1)
                                                                         << " " << chaser_ekf_state_.pos(2)  <<  std::endl;
    }
    std::cout << "[MIT-SLAM] iSAM estimated chaser position in world frame: \n" << chaser_est_state_.pos(0) << " " << chaser_est_state_.pos(1) << " " << chaser_est_state_.pos(2) << std::endl;
    if (params_.sim.compare("true") == 0) {
      std::cout << "[MIT-SLAM] Truth chaser attitude in world frame: \n" << chaser_gt_state_.quat(0) << " " << chaser_gt_state_.quat(1)  << " " << chaser_gt_state_.quat(2)  <<  " " << chaser_gt_state_.quat(3) << std::endl;
    }
    else {
      std::cout << "[MIT-SLAM] Truth chaser attitude in world frame: \n" << chaser_ekf_state_.quat(0) << " " << chaser_ekf_state_.quat(1)  << " " << chaser_ekf_state_.quat(2)  <<  " " << chaser_ekf_state_.quat(3) << std::endl;
    }

    std::cout << "[MIT-SLAM] iSAM estimated chaser attitude in world frame: \n" << chaser_est_state_.quat(0) << " " << chaser_est_state_.quat(1)  << " " << chaser_est_state_.quat(2)  <<  " " << chaser_est_state_.quat(3) << std::endl;
    if (params_.sim.compare("true") == 0) {
      std::cout << "[MIT-SLAM] Truth target attitude in world frame: \n" << target_gt_state_.quat(0) << " " << target_gt_state_.quat(1)  << " " << target_gt_state_.quat(2)  <<  " " << target_gt_state_.quat(3) << std::endl;
    }
    std::cout << "[MIT-SLAM] iSAM estimated target attitude in world frame: \n" << target_est_state_.quat(0) << " " << target_est_state_.quat(1)  << " " << target_est_state_.quat(2)  <<  " " << target_est_state_.quat(3) << std::endl;
    std::cout << "[MIT-SLAM] iSAM estimated chaser velocity in world frame: \n" << chaser_est_state_.vel(0) << " " << chaser_est_state_.vel(1) << " " << chaser_est_state_.vel(2) << std::endl;

    std::cout << std::endl;
    Eigen::Vector4f chaser_q_error;
    Eigen::Vector4f target_q_error;
    if (params_.sim.compare("true") == 0) {
      Eigen::Vector3f target_gt_omega_body;
      Eigen::Vector3f chaser_gt_omega_body;
      Eigen::Matrix3f target_gt_R = mit_slam::q2dcm(target_gt_state_.quat(0), target_gt_state_.quat(1), target_gt_state_.quat(2), target_gt_state_.quat(3));
      Eigen::Matrix3f chaser_gt_R = mit_slam::q2dcm(chaser_gt_state_.quat(0), chaser_gt_state_.quat(1), chaser_gt_state_.quat(2), chaser_gt_state_.quat(3));
      target_gt_omega_body = target_gt_R.transpose() * target_gt_state_.omega;
      chaser_gt_omega_body = chaser_gt_R.transpose() * chaser_gt_state_.omega;
      chaser_q_error = q_error(chaser_est_state_.quat, chaser_gt_state_.quat);
      target_q_error = q_error(target_est_state_.quat, target_gt_state_.quat);
      std::cout << "[MIT-SLAM] Chaser position error: \n" << chaser_est_state_.pos(0) - chaser_gt_state_.pos(0) << " "
                                                          << chaser_est_state_.pos(1) - chaser_gt_state_.pos(1) << " "
                                                          << chaser_est_state_.pos(2) - chaser_gt_state_.pos(2) << std::endl;
      std::cout << "[MIT-SLAM] Chaser velocity error: \n" << chaser_est_state_.vel(0) - chaser_gt_state_.vel(0) << " "
                                                          << chaser_est_state_.vel(1) - chaser_gt_state_.vel(1) << " "
                                                          << chaser_est_state_.vel(2) - chaser_gt_state_.vel(2) << std::endl;
      std::cout << "[MIT-SLAM] Chaser attitude error: \n" << chaser_q_error(0) << " "
                                                             << chaser_q_error(1) << " "
                                                             << chaser_q_error(2) << " "
                                                             << chaser_q_error(3) << std::endl;
      std::cout << "[MIT-SLAM] Target attitude error: \n" << target_q_error(0) << " "
                                                             << target_q_error(1) << " "
                                                             << target_q_error(2) << " "
                                                             << target_q_error(3) << std::endl;
      std::cout << std::endl;
      std::cout << "[MIT-SLAM] iSAM estimated chaser angular velocity \n" << chaser_est_state_.omega(0) << " " << chaser_est_state_.omega(1) << " " << chaser_est_state_.omega(2) << std::endl;
      std::cout << "[MIT-SLAM] iSAM estimated target angular velocity \n" << target_est_state_.omega(0) << " " << target_est_state_.omega(1) << " " << target_est_state_.omega(2) << std::endl;
      std::cout << std::endl;
      std::cout << "[MIT-SLAM] Chaser omega error: \n" << chaser_est_state_.omega(0) - chaser_gt_omega_body(0) << " "
                                                       << chaser_est_state_.omega(1) - chaser_gt_omega_body(1) << " "
                                                       << chaser_est_state_.omega(2) - chaser_gt_omega_body(2) << std::endl;
      std::cout << "[MIT-SLAM] Target omega error: \n" << target_est_state_.omega(0) - target_gt_omega_body(0) << " "
                                                       << target_est_state_.omega(1) - target_gt_omega_body(1) << " "
                                                       << target_est_state_.omega(2) - target_gt_omega_body(2) << std::endl;
    }
    else {
      Eigen::Vector3f chaser_ekf_omega_body;
      Eigen::Matrix3f chaser_ekf_R = mit_slam::q2dcm(chaser_ekf_state_.quat(0), chaser_ekf_state_.quat(1), chaser_ekf_state_.quat(2), chaser_ekf_state_.quat(3));
      chaser_ekf_omega_body = chaser_ekf_R.transpose() * chaser_ekf_state_.omega;
      chaser_q_error = q_error(chaser_est_state_.quat, chaser_ekf_state_.quat);
      std::cout << "[MIT-SLAM] Chaser position error: \n" << chaser_est_state_.pos(0) - chaser_ekf_state_.pos(0) << " "
                                                          << chaser_est_state_.pos(1) - chaser_ekf_state_.pos(1) << " "
                                                          << chaser_est_state_.pos(2) - chaser_ekf_state_.pos(2) << std::endl;
      std::cout << "[MIT-SLAM] Chaser velocity error: \n" << chaser_est_state_.vel(0) - chaser_ekf_state_.vel(0) << " "
                                                          << chaser_est_state_.vel(1) - chaser_ekf_state_.vel(1) << " "
                                                          << chaser_est_state_.vel(2) - chaser_ekf_state_.vel(2) << std::endl;
      std::cout << "[MIT-SLAM] Chaser attitude error: \n" << chaser_q_error(0) << " "
                                                             << chaser_q_error(1) << " "
                                                             << chaser_q_error(2) << " "
                                                             << chaser_q_error(3) << std::endl;
      std::cout << std::endl;
      std::cout << "[MIT-SLAM] iSAM estimated chaser angular velocity \n" << chaser_est_state_.omega(0) << " " << chaser_est_state_.omega(1) << " " << chaser_est_state_.omega(2) << std::endl;
      std::cout << "[MIT-SLAM] iSAM estimated target angular velocity \n" << target_est_state_.omega(0) << " " << target_est_state_.omega(1) << " " << target_est_state_.omega(2) << std::endl;
      std::cout << std::endl;
      std::cout << "[MIT-SLAM] Chaser omega error: \n" << chaser_est_state_.omega(0) - chaser_ekf_omega_body(0) << " "
                                                       << chaser_est_state_.omega(1) - chaser_ekf_omega_body(1) << " "
                                                       << chaser_est_state_.omega(2) - chaser_ekf_omega_body(2) << std::endl;
    }
}

void SlamNode::ChaserGtPoseCallback(const geometry_msgs::PoseStamped &msg) {
  chaser_gt_state_.quat << msg.pose.orientation.x,
                           msg.pose.orientation.y,
                           msg.pose.orientation.z,
                           msg.pose.orientation.w;
  chaser_gt_state_.pos << msg.pose.position.x,
                           msg.pose.position.y,
                           msg.pose.position.z;
}

void SlamNode::ChaserGtTwistCallback(const geometry_msgs::TwistStamped &msg) {
  chaser_gt_state_.vel << msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z;
  chaser_gt_state_.omega << msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z;
}

void SlamNode::TargetGtPoseCallback(const geometry_msgs::PoseStamped &msg) {
  target_gt_state_.quat << msg.pose.orientation.x,
                           msg.pose.orientation.y,
                           msg.pose.orientation.z,
                           msg.pose.orientation.w;
  target_gt_state_.pos << msg.pose.position.x,
                          msg.pose.position.y,
                          msg.pose.position.z;
}

void SlamNode::TargetGtTwistCallback(const geometry_msgs::TwistStamped &msg) {
  target_gt_state_.vel << msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z;
  target_gt_state_.omega << msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z;
}

void SlamNode::SetupBlobTracker(BlobTracker::Params params) {
  blob_tracker_ = std::unique_ptr<BlobTracker>(new BlobTracker(std::forward<BlobTracker::Params>(params)));
}

void SlamNode::SetupCloudOdometer(CloudOdometer::Params params) {
  cloud_odometer_ = std::unique_ptr<CloudOdometer>(new CloudOdometer(std::forward<CloudOdometer::Params>(params)));
}

void SlamNode::SetupGraphManager(GraphManager::Params params) {
  graph_manager_ = std::unique_ptr<GraphManager>(new GraphManager(std::forward<GraphManager::Params>(params)));
}


void SlamNode::PublishChaserPoseEst(const Eigen::Vector4f &quat,
                                    const Eigen::Vector3f &pos) {
  geometry_msgs::PoseWithCovariance p;
  p.pose.position.x = pos(0);
  p.pose.position.y = pos(1);
  p.pose.position.z = pos(2);
  p.pose.orientation.x = quat(0);
  p.pose.orientation.y = quat(1);
  p.pose.orientation.z = quat(2);
  p.pose.orientation.w = quat(3);

  // for now, don't use covariance
  int idx = 0;
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
        p.covariance[idx] = 0.0;
        idx++;
    }
  }
  chaser_est_pose_pub_.publish(p);
}

void SlamNode::PublishTargetPoseEst(const Eigen::Vector4f &quat,
                                    const Eigen::Vector3f &pos) {
  geometry_msgs::PoseWithCovariance p;
  p.pose.position.x = pos(0);
  p.pose.position.y = pos(1);
  p.pose.position.z = pos(2);
  p.pose.orientation.x = quat(0);
  p.pose.orientation.y = quat(1);
  p.pose.orientation.z = quat(2);
  p.pose.orientation.w = quat(3);
  int idx = 0;
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
        p.covariance[idx] = 0.0;
        idx++;
    }
  }
  target_est_pose_pub_.publish(p);
}

void SlamNode::PublishChaserTwistEst(const Eigen::Vector3f &chaser_vel,
                                    const Eigen::Vector3f &chaser_w) {
  geometry_msgs::TwistWithCovariance p;
  Eigen::Vector3f vel = chaser_vel;
  Eigen::Vector3f w = chaser_w;
  if (params_.ground) {
    vel(2) = 0.0;
    w(0) = 0.0;
    w(1) = 0.0;
  }
  p.twist.linear.x = vel(0);
  p.twist.linear.y = vel(1);
  p.twist.linear.z = vel(2);
  p.twist.angular.x = w(0);
  p.twist.angular.y = w(1);
  p.twist.angular.z = w(2);
  int row_cnt = 0;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
        p.covariance[j+row_cnt*6] = 0.0;
    }
    row_cnt++;
  }
  int next_row_cnt = 3;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
        p.covariance[j+3+next_row_cnt*6] = 0.0;
    }
    row_cnt++;
  }
  chaser_est_twist_pub_.publish(p);
}

void SlamNode::PublishTargetTwistEst(const Eigen::Vector3f &target_vel,
                                    const Eigen::Vector3f &target_w) {
  geometry_msgs::TwistWithCovariance p;
  Eigen::Vector3f vel = target_vel;
  Eigen::Vector3f w = target_w;
  if (params_.ground) {
    vel(2) = 0.0;
    w(0) = 0.0;
    w(1) = 0.0;
  }
  p.twist.linear.x = vel(0);
  p.twist.linear.y = vel(1);
  p.twist.linear.z = vel(2);
  p.twist.angular.x = w(0);
  p.twist.angular.y = w(1);
  p.twist.angular.z = w(2);
  int row_cnt = 0;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
        p.covariance[j+row_cnt*6] = 0.0;
    }
    row_cnt++;
  }
  int next_row_cnt = 3;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
        p.covariance[j+3+next_row_cnt*6] = 0.0;
    }
    row_cnt++;
  }
  target_est_twist_pub_.publish(p);
}

void SlamNode::PublishInertia(const Eigen::Matrix3f &J_est) {
  geometry_msgs::Inertia msg;
  msg.ixx = J_est(0,0);
  msg.ixy = J_est(0,1);
  msg.ixz = J_est(0,2);
  msg.iyy = J_est(1,1);
  msg.iyz = J_est(1,2);
  msg.izz = J_est(2,2);
  inertia_pub_.publish(msg);
}

void SlamNode::PublishGTPoseEst(const Eigen::Matrix4f &pose,
                                const std::string &frame_id) {
  geometry_msgs::PoseStamped p;
  p.pose.position.x = pose(0,3);
  p.pose.position.y = pose(1,3);
  p.pose.position.z = pose(2,3);
  Eigen::Matrix3f rot;
  rot << pose(0,0), pose(0,1), pose(0,2),
         pose(1,0), pose(1,1), pose(1,2),
         pose(2,0), pose(2,1), pose(2,2);
  Eigen::Vector4f quat = mit_slam::dcm2q(rot);
  p.pose.orientation.x = quat(0);
  p.pose.orientation.y = quat(1);
  p.pose.orientation.z = quat(2);
  p.pose.orientation.w = quat(3);

  p.header.frame_id = frame_id;
  est_GT_pose_pub_.publish(p);

}

void SlamNode::PublishGCPoseEst(const Eigen::Matrix4f &pose,
                                const std::string &frame_id) {
  geometry_msgs::PoseStamped p;
  p.pose.position.x = pose(0,3);
  p.pose.position.y = pose(1,3);
  p.pose.position.z = pose(2,3);
  Eigen::Matrix3f rot;
  rot << pose(0,0), pose(0,1), pose(0,2),
         pose(1,0), pose(1,1), pose(1,2),
         pose(2,0), pose(2,1), pose(2,2);
  Eigen::Vector4f quat = mit_slam::dcm2q(rot);
  p.pose.orientation.x = quat(0);
  p.pose.orientation.y = quat(1);
  p.pose.orientation.z = quat(2);
  p.pose.orientation.w = quat(3);

  p.header.frame_id = frame_id;
  est_GC_pose_pub_.publish(p);

}

void SlamNode::PublishtWTEst(const Eigen::Vector3f &t_WT,
                               const std::string &frame_id) {
  geometry_msgs::PointStamped p;
  p.header.frame_id = frame_id;
  p.point.x = t_WT(0);
  p.point.y = t_WT(1);
  p.point.z = t_WT(2);

  est_t_WT_pub_.publish(p);
}

void SlamNode::PublishCentroid(const Eigen::Vector3f &centroid,
                               const std::string &frame_id) {
  geometry_msgs::PointStamped p;
  p.header.frame_id = frame_id;
  p.point.x = centroid(0);
  p.point.y = centroid(1);
  p.point.z = centroid(2);

  centroid_pub_.publish(p);
}

void SlamNode::PublishDeltaPose(const Eigen::Matrix4f &pose,
                                const std::string &frame_id) {
  geometry_msgs::PoseStamped p;
  p.pose.position.x = pose(0,3);
  p.pose.position.y = pose(1,3);
  p.pose.position.z = pose(2,3);
  Eigen::Matrix3f rot;
  rot << pose(0,0), pose(0,1), pose(0,2),
         pose(1,0), pose(1,1), pose(1,2),
         pose(2,0), pose(2,1), pose(2,2);
  Eigen::Vector4f quat = mit_slam::dcm2q(rot);
  p.pose.orientation.x = quat(0);
  p.pose.orientation.y = quat(1);
  p.pose.orientation.z = quat(2);
  p.pose.orientation.w = quat(3);

  p.header.frame_id = frame_id;
  delta_pose_pub_.publish(p);

}

void SlamNode::PublishLoopDeltaPose(const Eigen::Matrix4f &pose,
                                const std::string &frame_id) {
  geometry_msgs::PoseStamped p;
  p.pose.position.x = pose(0,3);
  p.pose.position.y = pose(1,3);
  p.pose.position.z = pose(2,3);
  Eigen::Matrix3f rot;
  rot << pose(0,0), pose(0,1), pose(0,2),
         pose(1,0), pose(1,1), pose(1,2),
         pose(2,0), pose(2,1), pose(2,2);
  Eigen::Vector4f quat = mit_slam::dcm2q(rot);
  p.pose.orientation.x = quat(0);
  p.pose.orientation.y = quat(1);
  p.pose.orientation.z = quat(2);
  p.pose.orientation.w = quat(3);

  p.header.frame_id = frame_id;
  loop_delta_pose_pub_.publish(p);

}

void SlamNode::PublishChaserTransforms() {
  tf::Transform ekf_chaser;
  ekf_chaser.setOrigin(tf::Vector3(chaser_ekf_state_.pos(0), chaser_ekf_state_.pos(1), chaser_ekf_state_.pos(2)));
  ekf_chaser.setRotation(tf::Quaternion(chaser_ekf_state_.quat(0), chaser_ekf_state_.quat(1), chaser_ekf_state_.quat(2), chaser_ekf_state_.quat(3)));
  chaser_br_.sendTransform(tf::StampedTransform(ekf_chaser, ros::Time::now(), "map", "ekf_chaser"));

  tf::Transform chaser2hazcam;
  chaser2hazcam.setOrigin(tf::Vector3(params_.T_C2H(0,3), params_.T_C2H(1,3), params_.T_C2H(2,3)));
  chaser2hazcam.setRotation(tf::Quaternion(params_.q_C2H(0), params_.q_C2H(1), params_.q_C2H(2), params_.q_C2H(3)));
  chaser_br_.sendTransform(tf::StampedTransform(chaser2hazcam, ros::Time::now(), "ekf_chaser", "haz_cam"));
}

void SlamNode::PublishMatchPointCloud(const Eigen::MatrixXf pcd,
                                      const std::vector<std::pair<int,int>> matches,
                                      const std::string &frame_id) {
  Eigen::MatrixXf match_eigen_pcd(3, matches.size());
  for (size_t i = 0; i < matches.size(); i++) {
    match_eigen_pcd.col(i) = pcd.col(matches[i].second);
  }
  sensor_msgs::PointCloud2 match_pcd = CreatePointCloud2(match_eigen_pcd);
  match_pcd.header.frame_id = frame_id;
  match_point_cloud_pub_.publish(match_pcd);
}

void SlamNode::PublishEstPointCloud(const Eigen::Matrix4f &pose_odometry,
                                    const std::string &frame_id) {
  // compute "estimated" current point cloud using the previous frame and the estimated delta-pose
  Eigen::Matrix<float, 4, Eigen::Dynamic> prev_eigen_pcd_h;
  prev_eigen_pcd_h.resize(4, prev_eigen_pcd_.cols());
  std::cout << "\n ----- " << std::endl;
  std::cout << prev_eigen_pcd_h.cols();
  std::cout << prev_eigen_pcd_h.rows();
  std::cout << "----- \n\n";
  
  // prev_eigen_pcd_h.topRows(3) = prev_eigen_pcd_;
  prev_eigen_pcd_h.topRows(3) = prev_eigen_pcd_;
  prev_eigen_pcd_h.bottomRows(1) = Eigen::Matrix<float, 1, Eigen::Dynamic>::Ones(prev_eigen_pcd_.cols());
  Eigen::Matrix<float, 4, Eigen::Dynamic> current_h = pose_odometry * prev_eigen_pcd_h;
  Eigen::Matrix<float, 3, Eigen::Dynamic> current = current_h.topRows(3);
  sensor_msgs::PointCloud2 est_pcd = CreatePointCloud2(current);
  est_pcd.header.frame_id = frame_id;
  est_point_cloud_pub_.publish(est_pcd);
}

void SlamNode::PublishEstLoopPointCloud(const Eigen::Matrix4f &pose_odometry,
                                    const std::string &frame_id) {
  // compute "estimated" current point cloud using the previous frame and the estimated delta-pose
  Eigen::Matrix<float, 4, Eigen::Dynamic> prev_eigen_pcd_h;
  prev_eigen_pcd_h.resize(4, prev_loop_eigen_pcd_.cols());
  prev_eigen_pcd_h.topRows(3) = prev_loop_eigen_pcd_;
  prev_eigen_pcd_h.bottomRows(1) = Eigen::Matrix<float, 1, Eigen::Dynamic>::Ones(prev_loop_eigen_pcd_.cols());
  Eigen::Matrix<float, 4, Eigen::Dynamic> current_h = pose_odometry * prev_eigen_pcd_h;
  Eigen::Matrix<float, 3, Eigen::Dynamic> current = current_h.topRows(3);

  sensor_msgs::PointCloud2 est_loop_pcd = CreatePointCloud2(current);
  est_loop_pcd.header.frame_id = frame_id;
  est_loop_point_cloud_pub_.publish(est_loop_pcd);
}

}  // namespace slam
