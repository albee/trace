/**
 * @file SLAMNode.h
 * @brief ROS node for SLAM.
 * @date Dec 30, 2020
 * @authors tonio terán (teran@mit.edu), charles oestreich (coestrei@mit.edu), jessica todd (jetodd@mit.edu)
 * @Copyright 2020 Space Systems Laboratory, MIT
 */

#ifndef MIT_SLAM_SLAMNODE_H_
#define MIT_SLAM_SLAMNODE_H_

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/Inertia.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <ff_msgs/EkfState.h>
#include <tf/transform_broadcaster.h>
#include <boost/numeric/odeint/stepper/runge_kutta_dopri5.hpp>
#include <boost/numeric/odeint/algebra/vector_space_algebra.hpp>
#include <boost/numeric/odeint/integrate/integrate.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <fstream>
#include <cmath>
#include <exception>

#include <trace_msgs/TDSlamInfo.h>
#include "mit_slam/BlobTracker.h"
#include "mit_slam/CloudOdometer.h"
#include "mit_slam/GeometryUtils.h"
#include "mit_slam/GraphManager.h"
#include "mit_slam/MsgsUtils.h"
#include "mit_slam/ParamUtils.h"

#include "polhode_alignment/principal_axes_solver.h"
#include "polhode_alignment/geometry_utils.h"
#include "polhode_alignment/conic_fitting.h"
#include "polhode_alignment/rigid_body_rotation.h"
#include "sph-vertigo/PrincipalAxesOpt.h"

// Set type for state vector
typedef Eigen::Matrix<float, 1, 13> state_type;

using PlaneFit = std::pair<pao::ConicFit<double>, int>;

namespace mit_slam {

//! Ros node that encapsulates the SLAM system.
/*!
  Handles the message-triggered callbacks by instantiating a `GraphManager`
  object onto which to redirect the messages. Information fusion and inference
  is carried out by iSAM2 and GTSAM in the background.
 */
class SlamNode {
 public:
  /// Structure for bundling the `SlamNode`s parameters.
  struct Params {
    /// Robot namespace
    std::string sim = "false";
    /// Verbose console output setting
    bool verbose;
    /// Verbose console output setting for timing
    bool timing_verbose;

    bool viz_hardware;
    /// Setting for SLAM activation
    bool activate = false;
    /// Desired frequency at which to operate the ROS node [hz].
    double loop_rate;
    /// ISS or ground test session
    bool ground;

    /// frames before convergence
    int convergence_frames;

    /// Frames before estimating inertia
    int omega_meas_frames;

    int test_number;

    bool slam_state_mode;

    /// graph update rate
    float graph_dt;
    bool loop_closure_enable;

    bool use_raw_clouds;

    bool use_raw_imu;

    bool compute_inertia;

    bool pub_truth;

    bool slam_spoof;

    std::string pcd_frame_id;

    /// Default topic in which the IMU measurements will be received.
    std::string imu_topic;
    /// Default Topic in which the point clouds are being received.
    std::string pcd_topic;

    /// Default topic in which chaser ground truth pose is being received.
    std::string chaser_gt_pose_topic;
    /// Default topic in which chaser ground truth twist is being received.
    std::string chaser_gt_twist_topic;

    /// Default topic in which chaser ground truth pose is being received.
    std::string target_gt_pose_topic;
    /// Default topic in which chaser ground truth twist is being received.
    std::string target_gt_twist_topic;

    // Default chaser EKF topic
    std::string chaser_ekf_topic;

    /// Topic to output the target's centroid estimates.
    std::string centroid_out_topic;
    /// Topic to output HazCam/Teaser pose odometry
    std::string delta_pose_topic;
    std::string loop_delta_pose_topic;
    /// Topic to output the match point cloud
    std::string match_point_cloud_topic;
    /// Topic to output the estimated current point cloud based on delta-pose estimate
    std::string est_point_cloud_topic;
    /// Topic to output the estimated current point cloud based on delta-pose estimate
    std::string est_loop_point_cloud_topic;

    std::string inertia_topic;

    /// Estimate publishing topics
    std::string chaser_est_pose_topic;
    std::string chaser_est_twist_topic;
    std::string target_est_pose_topic;
    std::string target_est_twist_topic;

    /// Useful factor graph variables
    std::string est_GT_pose_topic;
    std::string est_GC_pose_topic;
    std::string est_t_WT_topic;

    /// Timing stat message
    std::string timing_info_topic;

    /// Match down-sampling threshold & criteria for loop closure
    float loop_max_rot;
    float loop_max_trans;
    int downsample_thresh;
    int loop_match_thresh;
    int loop_idx_hist;

    /// Target angular velocity smoothing forgetting factor (ground only for now)
    float ff;

    /// IMU dt
    double imu_dt;

    double ekf_dt;

    /// State propagator dt
    double prop_dt;
    Eigen::Matrix3f J_target;  // nominal target inertia
    Eigen::Matrix3f J_chaser;  // nominal chaser inertia

    /// Chaser estimated initial state w/r to world frame (world frame is centered at target COM)
    Eigen::Matrix4f T_WC0_iss;
    Eigen::Vector3f v_WC0;
    Eigen::Matrix4f T_WC0_ground;

    /// TVR frame centroid for ISS
    Eigen::Vector3f r_RI;
    Eigen::Vector3f r_TR;

    Eigen::Matrix3f R_TP;

    /// IMU transformation w/r to chaser (honey) body frame (astrobee/config/robot/honey.config)
    Eigen::Matrix4f T_C2I;

    /// HazCam transformation w/r to chaser (honey) body frame (astrobee/config/robot/honey.config)
    Eigen::Matrix4f T_C2H;
    Eigen::Vector4f q_C2H;

    /// Target ISS position (for debugging only)
    Eigen::Vector3f t_targ_ISS;
    Eigen::Vector3f t_targ_ground;


    /// IMU biases for ground testing
    float imu_bias_acc_x;
    float imu_bias_acc_y;
    float imu_bias_acc_z;

    float imu_bias_omega_x;
    float imu_bias_omega_y;
    float imu_bias_omega_z;

    float cloud_odom_bad_x;
    float cloud_odom_bad_y;
    float cloud_odom_bad_z;
    float cloud_odom_bad_alpha;
  };

  /// Internal copy of the node's parameters.
  Params params_;

  /// Default constructor.
  explicit SlamNode(ros::NodeHandle* nh, const Params &params);
  /// Default destructor.
  ~SlamNode();

  /// Initializes the `BlobTracker` object using specified parameters.
  void SetupBlobTracker(BlobTracker::Params params);
  /// Initializes the `CloudOdometer` object using specified parameters.
  void SetupCloudOdometer(CloudOdometer::Params params);
  /// Initializes the `CloudOdometer` object using specified parameters.
  void SetupGraphManager(GraphManager::Params params);

  /// Database of feature descriptors for possible loop closures
  std::vector<teaser::FPFHCloudPtr> feature_database_;

  /// Database of point cloud frames
  std::vector<teaser::PointCloud> cloud_database_;

  /// Database of point cloud frames in Eigen format
  std::vector<Eigen::MatrixXf> eigen_cloud_database_;

  /// Database of target omega measurements in G frame for inertia est
  std::vector<Eigen::Vector3f> omega_inertia_meas_;
  int omega_inertia_count_;

  /// Current frame index
  int frame_idx_;
  /// Loop closure index
  int loop_idx_;
  /// boolean for activating graph timer once we've had point cloud callbacks.
  bool pcds_available_;
  /// Number of matches stat before downsampling
  int num_matches_stat_;
  /// Truncated/downsampled pcd size
  int trunc_pcd_size_stat_;

  bool states_initialized_;

  bool dont_smooth_omega_;

  /// Target delta-pose (T_GC)
  Eigen::Matrix4f T_CiCj_;
  Eigen::Matrix4f T_HiHj_;

  /// Last frame's delta pose (used if current frame's is bad).
  Eigen::Matrix4f T_CiCj_prev_;

  /// matches
  std::vector<std::pair<int, int>> matches_final_;
  std::vector<std::pair<int, int>> matches_final_loop_;

  /// Loop closure count
  int loop_closure_count_;

  /// Loop closure delta-pose
  Eigen::Matrix4f T_CiCj_loop_;
  Eigen::Matrix4f T_HiHj_loop_;

  /// IMU data for chaser
  Eigen::Vector3f chaser_omega_;
  Eigen::Vector3f chaser_accel_;

  /// Loop closure success
  bool loop_closure_success_;

  /// Inertia est activate
  bool inertia_est_activate_;

  /// Status params for convergence and inertia estimated
  bool converged_;
  bool inertia_estimated_;

  double inertia_time_;

  /// Variable to hold cloud centroid
  Eigen::Vector3f centroid_H_;

  /// Variable to hold latest point cloud message
  sensor_msgs::PointCloud2 pcd_msg_;


  /// state estimate structure
  struct State {
    Eigen::Vector3f pos;
    Eigen::Vector3f vel;
    Eigen::Vector4f quat;
    Eigen::Vector3f omega;
  };

  /// Estimated world states (world frame is centered at target COM)
  State chaser_est_state_;
  State target_est_state_;

  /// Ground truth ISS states (for analysis only)
  State chaser_gt_state_;
  State target_gt_state_;

  /// Chaser EKF state
  State chaser_ekf_state_;

  /// Smoothing gamma term for target omega (ground only for now)
  float gamma_k_;
  float gamma_k_1_;
  Eigen::Vector3f prev_targ_omega_;

  /// Geometric frame definition. Chaser body pose w/r to geometric frame, which is defined by first centroid.
  Eigen::Matrix4f T_GC0_;

  /// Placeholder for Geometric to Target frame transformation. Constant. For now, supplied by simulator.
  Eigen::Matrix4f T_GT_;

  Eigen::Matrix3f R_GT_;

  double prop_t_;

  Eigen::Matrix4f T_GT_truth_;

  /// Utility function to reset the SLAM nodelet
  void Reset();

  /// Utility function for down-sampling matches
  std::vector<std::pair<int, int>> DownSampleMatches(
      const std::vector<std::pair<int, int>> correspondences);

  /// Utility function for checking for bad odometry measurements
  void CheckBadOdom();

  /// Utility function for attempting loop closures
  void AttemptLoopClosure();

  /// Utility function to write output to console
  void ConsoleOutput();

  Eigen::Matrix3d FindBodyFrame(const Eigen::Matrix3d& R_EG, const Eigen::MatrixXd& omegaB_G);

  std::vector<std::pair<pao::ConicFit<double>, int>>
  GetBestConicFitsConstrained(const Eigen::Matrix3d &R,
                              const Eigen::MatrixXd &omegaB_G) const;

  std::tuple<Eigen::Vector3d, Eigen::Vector3d> GetHypAxes(
      const Eigen::Matrix3d& R_EG, const std::vector<PlaneFit>& fits, const Eigen::MatrixXd& omegaB_G) const;



  std::tuple<Eigen::VectorXd, Eigen::MatrixXd> CreateCleanPolhode(size_t N);

  void WriteDataToDisk(size_t N, const Eigen::VectorXd& times,
                       const Eigen::MatrixXd& Bc_omega_tru,
                       const Eigen::MatrixXd& Bn_omega_tru,
                       const Eigen::MatrixXd& Gn_omega, const double rads,
                       const Eigen::Vector3d& axis,
                       const Eigen::AngleAxisd& aa_BG_est);



 private:
  // Factor graph update timer
  ros::Timer graph_timer_;
  void GraphUpdate(const ros::TimerEvent& t);

  // Inertia estimation timer
  ros::Timer inertia_est_timer_;
  void InertiaEstimate(const ros::TimerEvent& t);

  // Propagate timer
  ros::Timer prop_timer_;
  void Propagate(const ros::TimerEvent& t);
  boost::numeric::odeint::runge_kutta_dopri5<
      state_type, float, state_type, float,
      boost::numeric::odeint::vector_space_algebra>
      chaser_stepper_;
  boost::numeric::odeint::runge_kutta_dopri5<
      state_type, float, state_type, float,
      boost::numeric::odeint::vector_space_algebra>
      target_stepper_;

  // IMU measurements.

  /// Subscriber for the IMU information.
  ros::Subscriber imu_sub_;
  /// Callback for the IMU measurements.
  void ImuMeasCallback(const sensor_msgs::Imu &msg);

  // Point cloud measurements.

  /// Subscriber for the HazCam's point clouds.
  ros::Subscriber pcd_sub_;
  /// Callback for the point cloud measurements.
  void PointCloudCallback(const sensor_msgs::PointCloud2 &msg);

  /// Subscriber for chaser ground truth pose in ISS frame (evaluation analysis only)
  ros::Subscriber chaser_gt_pose_sub_;
  void ChaserGtPoseCallback(const geometry_msgs::PoseStamped &msg);
  /// Subscriber for chaser ground truth twist in ISS frame (evaluation analysis only)
  ros::Subscriber chaser_gt_twist_sub_;
  void ChaserGtTwistCallback(const geometry_msgs::TwistStamped &msg);

  /// Subscriber for target ground truth pose in ISS frame
  ros::Subscriber target_gt_pose_sub_;
  void TargetGtPoseCallback(const geometry_msgs::PoseStamped &msg);
  /// Subscriber for target ground truth twist in ISS frame
  ros::Subscriber target_gt_twist_sub_;
  void TargetGtTwistCallback(const geometry_msgs::TwistStamped &msg);

  ros::Subscriber chaser_ekf_sub_;
  void ChaserEKFCallback(const ff_msgs::EkfState::ConstPtr msg);

  tf::TransformBroadcaster chaser_br_;
  void PublishChaserTransforms();


  // Internal objects for estimation tasks.

  /// Blob tracker for range and bearing measurements to target's centroid.
  std::unique_ptr<BlobTracker> blob_tracker_ = nullptr;
  /// Cloud odometer.
  std::unique_ptr<CloudOdometer> cloud_odometer_ = nullptr;
  /// Graph manager.
  std::unique_ptr<GraphManager> graph_manager_ = nullptr;


  // Outputs and publishers.

  /// Publisher for the target's centroid estimate [geometry_msgs/PointStamped].
  ros::Publisher centroid_pub_;
  /// Assembles and published a stamped point using centroid estimate.
  void PublishCentroid(const Eigen::Vector3f &centroid,
                       const std::string &frame_id);

  /// Publishers for estimates
  ros::Publisher chaser_est_pose_pub_;
  void PublishChaserPoseEst(const Eigen::Vector4f &quat,
                            const Eigen::Vector3f &pos);
  ros::Publisher target_est_pose_pub_;
  void PublishTargetPoseEst(const Eigen::Vector4f &quat,
                            const Eigen::Vector3f &pos);
  ros::Publisher chaser_est_twist_pub_;
  void PublishChaserTwistEst(const Eigen::Vector3f &chaser_vel,
                             const Eigen::Vector3f &chaser_w);
  ros::Publisher target_est_twist_pub_;
  void PublishTargetTwistEst(const Eigen::Vector3f &target_vel,
                             const Eigen::Vector3f &target_w);

  /// Publishers for other useful factor graph variables
  ros::Publisher est_GT_pose_pub_;
  void PublishGTPoseEst(const Eigen::Matrix4f &pose,
                        const std::string &frame_id);

  ros::Publisher est_GC_pose_pub_;
  void PublishGCPoseEst(const Eigen::Matrix4f &pose,
                        const std::string &frame_id);

  ros::Publisher est_t_WT_pub_;
  void PublishtWTEst(const Eigen::Vector3f &t_WT,
                     const std::string &frame_id);

  /// Publisher for inertia tensor  estimate
  ros::Publisher inertia_pub_;
  void PublishInertia(const Eigen::Matrix3f &J_est);

  /// Publisher for the timing info message
  ros::Publisher timing_info_pub_;

  /// Publisher for match point cloud [sensor_msgs/PointCloud2].
  ros::Publisher match_point_cloud_pub_;
  /// Passes on matches to the match point cloud publisher.
  void PublishMatchPointCloud(const Eigen::MatrixXf pcd,
                              const std::vector<std::pair<int, int>> matches,
                              const std::string &frame_id);
  /// Publisher for estimated point cloud via delta-poses
  ros::Publisher est_point_cloud_pub_;
  /// Previous point cloud in Eigen format needed for estimation
  Eigen::MatrixXf prev_eigen_pcd_;
  /// Passes on an odometry estimate to the estimated point cloud publisher.
  void PublishEstPointCloud(const Eigen::Matrix4f &pose_odometry,
                            const std::string &frame_id);

  /// Publisher for estimated point cloud via delta-poses
  ros::Publisher est_loop_point_cloud_pub_;
  /// Previous point cloud in Eigen format needed for estimation
  Eigen::MatrixXf prev_loop_eigen_pcd_;
  /// Passes on an odometry estimate to the estimated point cloud publisher.
  void PublishEstLoopPointCloud(const Eigen::Matrix4f &pose_odometry,
                            const std::string &frame_id);

  ros::Publisher delta_pose_pub_;
  void PublishDeltaPose(const Eigen::Matrix4f &pose,
                        const std::string &frame_id);
  ros::Publisher loop_delta_pose_pub_;
  void PublishLoopDeltaPose(const Eigen::Matrix4f &pose,
                            const std::string &frame_id);

  // dynamic propagator stuff
  class dynamic_step_ {
    Eigen::Matrix3f J_param;

   public:
    dynamic_step_(Eigen::Matrix3f G) : J_param(G) {}

    void operator()(state_type const &x, state_type &dx, const float &t) const {
      Eigen::Vector3f r = x.segment(0, 3);
      Eigen::Vector3f v = x.segment(3, 3);
      Eigen::Vector4f q = x.segment(6, 4);
      Eigen::Vector3f w = x.segment(10, 3);
      Eigen::Vector3f M =
          Eigen::VectorXf::Zero(3);  // Assuming no external torques on target
      Eigen::Matrix3f J_inv;

      J_inv = J_param.inverse();

      Eigen::Vector3f rdot = v;
      Eigen::Vector3f vdot;
      vdot << 0.0, 0.0, 0.0;

      // Quaternion kinematics
      Eigen::MatrixXf B(4, 4);
      B(0, 0) = 0.0;
      B(0, 1) = w(2);
      B(0, 2) = -w(1);
      B(0, 3) = w(0);

      B(1, 0) = -w(2);
      B(1, 1) = 0.0;
      B(1, 2) = w(0);
      B(1, 3) = w(1);

      B(2, 0) = w(1);
      B(2, 1) = -w(0);
      B(2, 2) = 0.0;
      B(2, 3) = w(2);

      B(3, 0) = -w(0);
      B(3, 1) = -w(1);
      B(3, 2) = -w(2);
      B(3, 3) = 0.0;

      Eigen::Vector4f qdot = 0.5 * B * q;

      // Euler's equations for rigid bodies
      Eigen::Vector3f wdot = J_inv * (M - w.cross(J_param * w));

      dx(0) = rdot(0);
      dx(1) = rdot(1);
      dx(2) = rdot(2);
      dx(3) = vdot(0);
      dx(4) = vdot(1);
      dx(5) = vdot(2);
      dx(6) = qdot(0);
      dx(7) = qdot(1);
      dx(8) = qdot(2);
      dx(9) = qdot(3);
      dx(10) = wdot(0);
      dx(11) = wdot(1);
      dx(12) = wdot(2);
    }
  };

  class dynamic_step_ground_ {
    Eigen::Matrix3f J_param;

   public:
    dynamic_step_ground_(Eigen::Matrix3f G) : J_param(G) {}

    void operator()(state_type const &x, state_type &dx, const float &t) const {
      Eigen::Vector3f r = x.segment(0, 3);
      Eigen::Vector3f v = x.segment(3, 3);
      Eigen::Vector4f q = x.segment(6, 4);
      Eigen::Vector3f w = x.segment(10, 3);

      Eigen::Vector3f rdot = v;
      Eigen::Vector3f vdot;
      vdot << 0.0, 0.0, 0.0;

      // Quaternion kinematics
      Eigen::MatrixXf B(4, 4);
      B(0, 0) = 0.0;
      B(0, 1) = w(2);
      B(0, 2) = -w(1);
      B(0, 3) = w(0);

      B(1, 0) = -w(2);
      B(1, 1) = 0.0;
      B(1, 2) = w(0);
      B(1, 3) = w(1);

      B(2, 0) = w(1);
      B(2, 1) = -w(0);
      B(2, 2) = 0.0;
      B(2, 3) = w(2);

      B(3, 0) = -w(0);
      B(3, 1) = -w(1);
      B(3, 2) = -w(2);
      B(3, 3) = 0.0;

      Eigen::VectorXf qdot = 0.5 * B * q;

      // Euler's equations for rigid bodies
      Eigen::Vector3f wdot;
      wdot << 0.0, 0.0, 0.0;

      dx(0) = rdot(0);
      dx(1) = rdot(1);
      dx(2) = rdot(2);
      dx(3) = vdot(0);
      dx(4) = vdot(1);
      dx(5) = vdot(2);
      dx(6) = qdot(0);
      dx(7) = qdot(1);
      dx(8) = qdot(2);
      dx(9) = qdot(3);
      dx(10) = wdot(0);
      dx(11) = wdot(1);
      dx(12) = wdot(2);
    }
  };
};

}  // namespace mit_slam

#endif  // MIT_SLAM_SLAMNODELET_H_
