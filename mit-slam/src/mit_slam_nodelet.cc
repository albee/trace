/**
 * @file mit_slam_nodelet.cpp
 * @brief Nodelet wrapper for TRACE SLAM node.
 * @date Jan 5, 2021
 * @authors tonio terán (teran@mit.edu), charles oestreich (coestrei@mit.edu), jessica todd (jetodd@mit.edu)
 * @Copyright 2021 Space Systems Laboratory, MIT
 */

#include <memory>
#include <thread>

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <ff_util/ff_nodelet.h>

#include <trace_msgs/TDStatus.h>
#include "mit_slam/SlamNode.h"
#include "mit_slam/ParamUtils.h"

namespace mit_slam {

  class MitSlamNodelet : public ff_util::FreeFlyerNodelet {
   public:
    MitSlamNodelet() : ff_util::FreeFlyerNodelet(true) {}
    ~MitSlamNodelet() {}

   private:
    ros::Subscriber sub_status_;
    std::string sim_ = "false";
    bool activate_ = false;
    bool viz_hardware_ = false;
    bool slam_spoof_ = false;
    int test_number_;
    std::string state_mode_ = "ekf";

    std::shared_ptr<std::thread> thread_;
    std::unique_ptr<mit_slam::SlamNode> slam_node_;
    double loop_rate_;

    void Initialize(ros::NodeHandle* nh) {
      ros::param::getCached("/td/sim", sim_);

      // Activate the main SLAM pipeline
      mit_slam::SlamNode::Params slam_params = SlamParamsFromRos();
      slam_node_ = std::unique_ptr<SlamNode>(new SlamNode(
          std::forward<ros::NodeHandle*>(this->GetPlatformHandle(true)),
          std::forward<SlamNode::Params>(slam_params)));

      // Activate the blob tracker.
      mit_slam::BlobTracker::Params blob_params = mit_slam::BlobParamsFromRos();
      slam_node_->SetupBlobTracker(blob_params);

      // Activate the cloud odometer.
      mit_slam::CloudOdometer::Params cloud_params =
          mit_slam::CloudParamsFromRos();
      slam_node_->SetupCloudOdometer(cloud_params);

      // Activate the graph manager
      mit_slam::GraphManager::Params graph_params;
      graph_params = mit_slam::GraphParamsFromRos();
      slam_node_->SetupGraphManager(graph_params);

      ros::param::getCached("/td/mit_slam/loop_rate", loop_rate_);
      ros::param::getCached("/td/mit_slam/viz_hardware", viz_hardware_);

      thread_.reset(new std::thread(&mit_slam::MitSlamNodelet::Run, this));

      
      sub_status_ = nh->subscribe<trace_msgs::TDStatus>(
          "td/status", 5, &MitSlamNodelet::StatusCallback, this);
      }

      void Run() {
        NODELET_INFO_STREAM("[MIT-SLAM] Initialized.");
        ros::Rate ros_rate(loop_rate_);

        if (viz_hardware_) {
          activate_ = true;
        }

        while (ros::ok()) {
          slam_node_->params_.activate = activate_;
          slam_node_->params_.slam_spoof = slam_spoof_;
          slam_node_->params_.test_number = test_number_;
          if (state_mode_.compare("slam") == 0) {
            slam_node_->params_.slam_state_mode = true;
          } else {
            slam_node_->params_.slam_state_mode = false;
          }
          ros::spinOnce();
          ros_rate.sleep();
        }
      }

      void StatusCallback(const trace_msgs::TDStatus::ConstPtr& msg) {
        activate_ = msg->slam_activate;
        slam_spoof_ = msg->test_slam_spoof;
        test_number_ = msg->test_number;
        state_mode_ = msg->test_state_mode;
      }

      SlamNode::Params SlamParamsFromRos() {
        SlamNode::Params params;

        ros::param::getCached("/td/sim", params.sim);

        ros::param::getCached("/td/mit_slam/verbose", params.verbose);
        ros::param::getCached("/td/mit_slam/timing_verbose", params.timing_verbose);
        ros::param::getCached("/td/mit_slam/viz_hardware", params.viz_hardware);
        ros::param::getCached("/td/mit_slam/loop_rate", params.loop_rate);  // [Hz]
        ros::param::getCached("/td/mit_slam/slam_ground", params.ground);
        ros::param::getCached("/td/mit_slam/convergence_frames", params.convergence_frames);
        ros::param::getCached("/td/mit_slam/omega_meas_frames", params.omega_meas_frames);
        ros::param::getCached("/td/mit_slam/graph_dt", params.graph_dt);
        ros::param::getCached("/td/mit_slam/loop_closure_enable", params.loop_closure_enable);
        ros::param::getCached("/td/mit_slam/use_raw_clouds", params.use_raw_clouds);
        ros::param::getCached("/td/mit_slam/use_raw_imu", params.use_raw_imu);
        ros::param::getCached("/td/mit_slam/pub_truth", params.pub_truth);
        ros::param::getCached("/td/mit_slam/compute_inertia", params.compute_inertia);
        /// Default ROS topic names for subscribing
        if (params.sim.compare("true") == 0) {
          params.target_gt_pose_topic = "/bumble/loc/truth/pose";
          params.target_gt_twist_topic = "/bumble/loc/truth/twist";
          params.chaser_gt_pose_topic = "loc/truth/pose";
          params.chaser_gt_twist_topic = "loc/truth/twist";
        }
        params.chaser_ekf_topic = "gnc/ekf";

        params.imu_topic = "hw/imu";
        params.pcd_topic = "hw/depth_haz/points";
        params.slam_spoof = slam_spoof_;
        params.test_number = test_number_;
        if (state_mode_.compare("slam") == 0) {
          params.slam_state_mode = true;
        } else {
          params.slam_state_mode = false;
        }

        float r_RI_x;
        float r_RI_y;
        float r_RI_z;
        if (!params.ground) {
          ros::param::getCached("/td/r_RI_ISS_x", r_RI_x);
          ros::param::getCached("/td/r_RI_ISS_y", r_RI_y);
          ros::param::getCached("/td/r_RI_ISS_z", r_RI_z);
          params.r_RI << r_RI_x, r_RI_y, r_RI_z;
        }
        int test_num = 0;
        ros::param::getCached("/td/gds_test_num", test_num);
        std::string test_number_str = std::to_string(test_num);
        if (!params.ground) {
          if (test_num > 100) {
            if (test_number_str[0] == '1') {
              params.r_TR << 0.15, 0.0, 0.0;
            }
            if (test_number_str[0] == '2') {
              params.r_TR << 0.0, -1.5, 0.0;
              float temp = params.r_RI(1);
              params.r_RI(1) = temp + params.r_TR(1);
            }
            if (test_number_str[0] == '3') {
              params.r_TR << 0.0, 0.0, 0.25;
            }
          } else {
            params.r_TR << 0.0, -1.5, 0.0;
            float temp = params.r_RI(1);
            params.r_RI(1) = temp + params.r_TR(1);
          }
        }


        /// Default ROS topic names for publishing
        params.centroid_out_topic = "/td/mit_slam/target_centroid";
        params.match_point_cloud_topic = "/td/mit_slam/match_point_cloud";
        params.est_point_cloud_topic = "/td/mit_slam/est_point_cloud";
        params.est_loop_point_cloud_topic = "/td/mit_slam/est_loop_point_cloud";
        params.delta_pose_topic = "/td/mit_slam/delta_pose";
        params.loop_delta_pose_topic = "/td/mit_slam/loop_delta_pose";
        params.chaser_est_pose_topic = "/td/mit_slam/chaser_pose";
        params.chaser_est_twist_topic = "/td/mit_slam/chaser_twist";
        params.target_est_pose_topic = "/td/mit_slam/target_pose";
        params.target_est_twist_topic = "/td/mit_slam/target_twist";
        params.est_GT_pose_topic = "/td/mit_slam/pose_GT";
        params.est_GC_pose_topic = "/td/mit_slam/pose_GC";
        params.est_t_WT_topic = "/td/mit_slam/t_WT";
        params.timing_info_topic = "/td/mit_slam/timing_info";
        params.inertia_topic = "/td/mit_slam/inertia";


        ros::param::getCached("/td/mit_slam/downsample_thresh", params.downsample_thresh);
        ros::param::getCached("/td/mit_slam/loop_match_thresh", params.loop_match_thresh);
        ros::param::getCached("/td/mit_slam/loop_max_rot", params.loop_max_rot);
        ros::param::getCached("/td/mit_slam/loop_max_trans", params.loop_max_trans);
        ros::param::getCached("/td/mit_slam/loop_idx_hist", params.loop_idx_hist);

        ros::param::getCached("/td/mit_slam/ff", params.ff);

        ros::param::getCached("/td/mit_slam/imu_dt", params.imu_dt);  // [s]
        ros::param::getCached("/td/mit_slam/prop_dt", params.prop_dt);
        ros::param::getCached("/td/mit_slam/ekf_dt", params.ekf_dt);

        // Nominal target inertia
        std::vector<double> J_vec_target;
        ros::param::getCached("/td/mit_slam/target_J", J_vec_target);
        int k = 0;
        for (int i = 0; i < 3; i++) {
          for (int j = 0; j < 3; j++) {
            params.J_target(i, j) = J_vec_target[k];
            k++;
          }
        }

        // Nominal chaser inertia
        std::vector<double> J_vec_chaser;
        ros::param::getCached("/td/mit_slam/chaser_J", J_vec_chaser);
        k = 0;
        for (int i = 0; i < 3; i++) {
          for (int j = 0; j < 3; j++) {
            params.J_chaser(i, j) = J_vec_chaser[k];
            k++;
          }
        }

        // Initial chaser pose estimate (iss)
        std::vector<double> T_WC0_ros_iss;
        ros::param::getCached("/td/mit_slam/T_WC0_iss", T_WC0_ros_iss);
        k = 0;
        for (int i = 0; i < 4; i++) {
          for (int j = 0; j < 4; j++) {
            params.T_WC0_iss(i, j) = T_WC0_ros_iss[k];
            k++;
          }
        }

         // Initial chaser pose estimate (ground)
        std::vector<double> T_WC0_ros_ground;
        ros::param::getCached("/td/mit_slam/T_WC0_ground", T_WC0_ros_ground);
        k = 0;
        for (int i = 0; i < 4; i++) {
          for (int j = 0; j < 4; j++) {
            params.T_WC0_ground(i, j) = T_WC0_ros_ground[k];
            k++;
          }
        }

        // Initial chaser velocity estimate
        std::vector<double> v_WC0_ros;
        ros::param::getCached("/td/mit_slam/v_WC0", v_WC0_ros);
        params.v_WC0 << v_WC0_ros[0], v_WC0_ros[1], v_WC0_ros[2];

        ros::param::getCached("/td/mit_slam/cloud_odom_bad_x", params.cloud_odom_bad_x);
        ros::param::getCached("/td/mit_slam/cloud_odom_bad_y", params.cloud_odom_bad_y);
        ros::param::getCached("/td/mit_slam/cloud_odom_bad_z", params.cloud_odom_bad_z);
        ros::param::getCached("/td/mit_slam/cloud_odom_bad_alpha", params.cloud_odom_bad_alpha);

        /// Get IMU bias parameters
        if (params.ground) {
          ros::param::getCached("/td/mit_slam/imu_bias_acc_x_ground", params.imu_bias_acc_x);
          ros::param::getCached("/td/mit_slam/imu_bias_acc_y_ground", params.imu_bias_acc_y);
          ros::param::getCached("/td/mit_slam/imu_bias_acc_z_ground", params.imu_bias_acc_z);
          ros::param::getCached("/td/mit_slam/imu_bias_omega_x_ground", params.imu_bias_omega_x);
          ros::param::getCached("/td/mit_slam/imu_bias_omega_y_ground", params.imu_bias_omega_y);
          ros::param::getCached("/td/mit_slam/imu_bias_omega_z_ground", params.imu_bias_omega_z);

          // remove weird sim IMU bias for ground hardware tests.
          if (params.sim.compare("true") != 0) {
            params.imu_bias_acc_x = 0.0;
            params.imu_bias_acc_y = 0.0;
            params.imu_bias_omega_z = 0.0;
          }
        } else {
          ros::param::getCached("/td/mit_slam/imu_bias_acc_x_iss", params.imu_bias_acc_x);
          ros::param::getCached("/td/mit_slam/imu_bias_acc_y_iss", params.imu_bias_acc_y);
          ros::param::getCached("/td/mit_slam/imu_bias_acc_z_iss", params.imu_bias_acc_z);
          ros::param::getCached("/td/mit_slam/imu_bias_omega_x_iss", params.imu_bias_omega_x);
          ros::param::getCached("/td/mit_slam/imu_bias_omega_y_iss", params.imu_bias_omega_y);
          ros::param::getCached("/td/mit_slam/imu_bias_omega_z_iss", params.imu_bias_omega_z);
        }

        /// Non-changing parameters (no ROS param setting)
        // Chaser to IMU transform
        params.T_C2I << 0, -1, 0, 0.0247,
                        1,  0, 0, 0.0183,
                        0,  0, 1, 0.0094,
                        0,  0, 0,      1;

        // Use calibrated b-sharp IMU transform
        if (params.sim.compare("true") != 0 && params.ground) {
          params.T_C2I << -0.1274278, -0.9918453, -0.0022533, 0.0247,
                           0.9917291, -0.1273771, -0.0157628, 0.0183,
                          0.0153472, -0.0042433,  0.9998732, 0.0094,
                          0,  0, 0,      1;
        }

        // Chaser to HazCam transform
        params.T_C2H << 0, 0,  1,  0.1328,
                        -1, 0, 0,  0.0362,
                        0, -1,  0, -0.0826,
                        0,  0,  0,       1;

        // Rotation from Envisat principal axes to our target body frame convention
        params.R_TP << -0.019026452224050, 0.005224221963522, -0.999805331862479,
                        0.077059267410756, 0.997019487094011, 0.003743215317479,
                                0.996844954555062, -0.076973046319288, -0.019372318354941;

        params.q_C2H = dcm2q(params.T_C2H.block(0, 0, 3, 3));

        // Target truth position
        params.t_targ_ISS << 10.9, -6.65, 4.9;
        params.t_targ_ground << 0.0, -0.5, -0.7;


        return params;
      }
  };

  }  // namespace mit_slam

// Declare the plugin
PLUGINLIB_EXPORT_CLASS(mit_slam::MitSlamNodelet, nodelet::Nodelet);
