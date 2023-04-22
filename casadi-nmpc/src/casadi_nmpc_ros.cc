/**
 * @file casadi_nmpc_ros.cc
 * @author Keenan Albee
 * @brief ROS interface (pubs and subs)of casadi_nmpc.
 * @version 0.1
 * @date 2023-04-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "casadi_nmpc/casadi_nmpc.h"


namespace casadi_nmpc{
/* ************************************************************************** */
  void CasadiNMPCNodelet::status_callback(const trace_msgs::TDStatus::ConstPtr& msg) {
    control_mode_ = msg->td_control_mode;
    state_mode_ = msg->td_state_mode;
    gain_mode_ = msg->gain_mode;
    online_update_mode_ = msg->online_update_mode;

    if (msg->casadi_on_target != casadi_on_target_) {  // update if different
      casadi_on_target_ = msg->casadi_on_target;
      CasadiNMPCNodelet::get_regulation_and_uc_bound_parameters();
    }
    switch_gains(gain_mode_);  // set the desired gains
  }

  /* ************************************************************************** */
  void CasadiNMPCNodelet::w_bound_callback(const std_msgs::Float64MultiArray::ConstPtr uc_bound) {
    /* Get the latest w_bound/uc_bound from Charles. Must be called before tube MPC starts!
    */
    int i = uc_bound->layout.dim[0].size;
    int j = uc_bound->layout.dim[1].size;
    std::vector<double> data = uc_bound->data; // copy constructor of underlying data
    MatrixXd w_bound_charles_(4, 3);
    w_bound_charles_ = Eigen::Map<Eigen::MatrixXd>(data.data(), i, j);  // std::vector --> Eigen
    w_bound_ << w_bound_charles_(0), w_bound_charles_(1), w_bound_charles_(2), w_bound_charles_(6), w_bound_charles_(7), w_bound_charles_(8);

    prepare_mrpi(w_bound_, u_mag_, MPC_dt_, mass_, Q_pos_anc_factor_, Q_vel_anc_factor_, R_anc_factor_);  // get K_dr and Au and bu ready
  }

  /* ************************************************************************** */
  void CasadiNMPCNodelet::inertia_callback(const geometry_msgs::Inertia::ConstPtr inertia_msg) {
    J_targ_(0,0) = inertia_msg->ixx;
    J_targ_(0,1) = inertia_msg->ixy;
    J_targ_(0,2) = inertia_msg->ixz;
    J_targ_(1,1) = inertia_msg->iyy;
    J_targ_(1,2) = inertia_msg->iyz;
    J_targ_(2,2) = inertia_msg->izz;
    J_targ_(1,0) = J_targ_(0,1);
    J_targ_(2,1) = J_targ_(1,2);
    J_targ_(2,0) = J_targ_(0,2);
  }

  /* ************************************************************************** */
  void CasadiNMPCNodelet::x_des_traj_callback(const std_msgs::Float64MultiArray::ConstPtr msg) {
    /* The `td/tube_mpc/traj` subscriber callback
    Called once by chaser_coordinator_nodelet. This is the full reference trajectory.

    x_des_traj = [t, position, velocity, linear accel, jerk]
    */
    // Make sure to do this only once.
    if (HAVE_TRAJ != 1) {
        NODELET_INFO_STREAM("x_des_traj callback called...");
        int i = msg->layout.dim[0].size;
        int j = msg->layout.dim[1].size;
        std::vector<double> data = msg->data; // copy constructor of underlying data
        // NOTE: default ROS message conversion flips row and column order!!!
        eigen_x_des_traj_ = Eigen::Map<Eigen::MatrixXd>(data.data(), j, i).transpose();  // std::vector --> Eigen
        eigen_x_des_traj_init_ = Eigen::Map<Eigen::MatrixXd>(data.data(), j, i).transpose();  // std::vector --> Eigen
        N_traj_ = i;

        NODELET_INFO_STREAM(eigen_x_des_traj_.rows() << " " << eigen_x_des_traj_.cols());
        IOFormat HeavyFmt(FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");
        std::string sep = "\n----------------------------------------\n";
        NODELET_INFO_STREAM(sep << eigen_x_des_traj_.row(0).format(HeavyFmt) << sep);
        NODELET_INFO_STREAM("...traj read-in complete.");

        dt_traj_ = eigen_x_des_traj_(1, 0) - eigen_x_des_traj_(0, 0);
        traj_rate_ = 1.0/dt_traj_;
        HAVE_TRAJ = 1;
    }
  }

  /* ************************************************************************** */
  void CasadiNMPCNodelet::x_des_traj_body_callback(const trace_msgs::TDTrajBody::ConstPtr msg) {
    /* Get body traj and nominal target orientation.
    */
    int i = msg->traj_body.layout.dim[0].size;
    int j = msg->traj_body.layout.dim[1].size;

    std::vector<double> data = msg->traj_body.data;
    eigen_x_des_traj_body_ = Eigen::Map<Eigen::MatrixXd>(data.data(), j, i).transpose();

    int size = msg->q_targ_0_hist.size();
    eigen_q_targ_0_hist_ = Eigen::MatrixXd::Zero(size, 4);
    for (int idx=0; idx < size; idx++) {
      geometry_msgs::Quaternion quat = msg->q_targ_0_hist[idx];
      eigen_q_targ_0_hist_(idx, 0) = quat.x;
      eigen_q_targ_0_hist_(idx, 1) = quat.y;
      eigen_q_targ_0_hist_(idx, 2) = quat.z;
      eigen_q_targ_0_hist_(idx, 3) = quat.w;
    }
  }

  /*
  ** Pub and subs
  */

  /* ************************************************************************** */
  void CasadiNMPCNodelet::publish_casadi_status( ) {
    trace_msgs::TDCasadiStatus msg;
    msg.stamp = ros::Time::now();
    msg.chaser_coord_ok = chaser_coord_ok_;
    msg.mrpi_finished = mrpi_finished_;
    msg.traj_finished = traj_finished_;
    pub_casadi_status_.publish(msg);
  }

  /* ************************************************************************** */
  void CasadiNMPCNodelet::publish_mrpi( ) {
    trace_msgs::TDMRPI_msg mrpi_msg;

    std_msgs::Float64MultiArray K_msg;
    tf::matrixEigenToMsg(K_dr_, K_msg);  // Eigen --> msg

    std_msgs::Float64MultiArray Au_msg;
    tf::matrixEigenToMsg(Au_, Au_msg);  // Eigen --> msg

    std_msgs::Float64MultiArray bu_msg;
    tf::matrixEigenToMsg(bu_, bu_msg);  // Eigen --> msg

    std_msgs::Float64MultiArray AZ_msg;
    tf::matrixEigenToMsg(AZ_, AZ_msg);  // Eigen --> msg

    std_msgs::Float64MultiArray bZ_msg;
    tf::matrixEigenToMsg(bZ_, bZ_msg);  // Eigen --> msg

    std_msgs::Bool fallback_msg;
    fallback_msg.data = using_fallback_mrpi_;

    mrpi_msg.K  = K_msg;
    mrpi_msg.Au = Au_msg;
    mrpi_msg.bu = bu_msg;
    mrpi_msg.AZ = AZ_msg;
    mrpi_msg.bZ = bZ_msg;
    mrpi_msg.using_fallback_mrpi = fallback_msg;

    pub_mrpi_.publish(mrpi_msg);
  }

  /* ************************************************************************** */
  void CasadiNMPCNodelet::publish_eigen_x_des_traj() {
    std_msgs::Float64MultiArray eigen_x_des_traj_msg;
    tf::matrixEigenToMsg(eigen_x_des_traj_, eigen_x_des_traj_msg);  // Eigen --> msg

    pub_eigen_x_des_traj_.publish(eigen_x_des_traj_msg);
  }

  /* ************************************************************************** */
  void CasadiNMPCNodelet::publish_debug(Matrix<double, 6, 1> u_t_idx, Vector3d u0_mpc, Vector3d u0_dr,
    Matrix<double, 6, 1> x_nom) {
    trace_msgs::TDCasadiDebug msg;
    msg.header.stamp = ros::Time::now();

    // Translation control (body frame)
    msg.wrench.force.x = u_t_idx[0];
    msg.wrench.force.y = u_t_idx[1];
    msg.wrench.force.z = u_t_idx[2];  // 3D only

    msg.wrench.torque.x = u_t_idx[3];  // 3D only
    msg.wrench.torque.y = u_t_idx[4];  // 3D only
    msg.wrench.torque.z = u_t_idx[5];

    msg.u0_mpc.x = u0_mpc[0];
    msg.u0_mpc.y = u0_mpc[1];
    msg.u0_mpc.z = u0_mpc[2];

    msg.u0_dr.x = u0_dr[0];
    msg.u0_dr.y = u0_dr[1];
    msg.u0_dr.z = u0_dr[2];

    std_msgs::Float64MultiArray x_nom_msg;
    tf::matrixEigenToMsg(x_nom, x_nom_msg);  // Eigen --> msg
    msg.x_nom = x_nom_msg;

    msg.casadi_comp_time.data = casadi_comp_time_;
    msg.total_comp_time.data = total_comp_time_;

    msg.control_mode.data = control_mode_;
    msg.state_mode.data = state_mode_;

    msg.Q_pos_factor = Q1;
    msg.Q_vel_factor = Q4;
    msg.R_factor = R1;
    msg.QN_pos_factor = QN1;
    msg.QN_vel_factor = QN4;

    msg.Q_pos_tube_factor = Q1_T_;
    msg.Q_vel_tube_factor = Q4_T_;
    msg.R_tube_factor = R1_T_;
    msg.QN_pos_tube_factor = QN1_T_;
    msg.QN_vel_tube_factor = QN4_T_;

    msg.Q_pos_anc_factor = Q_pos_anc_factor_;
    msg.Q_vel_anc_factor = Q_vel_anc_factor_;
    msg.R_anc_factor = R_anc_factor_;

    msg.T = T;
    msg.N = N;
    msg.control_dt = control_dt_;

    pub_debug_.publish(msg);
  }


  /* ************************************************************************** */
  void CasadiNMPCNodelet::publish_ctl(tf2::Vector3 F_xyz_B, tf2::Vector3 T_xyz_B) {
    /* Publishes caclulated forces/torques to the FAM.

    Inputs:
    F_xyz_B: translational forces in the BODY frame
    T_xyz_B: attitude torques in the BODY frame

    Outputs:
    Publishes forces/torques on gnc/ctl/command at 62.5 Hz BODY frame
    */
    ff_msgs::FamCommand famcmd;

    famcmd.header.stamp = ros::Time::now();

    // Translation control (body frame)
    famcmd.wrench.force.x = F_xyz_B[0];
    famcmd.wrench.force.y = F_xyz_B[1];
    famcmd.wrench.force.z = F_xyz_B[2];  // 3D only

    // Attitude control (body frame)
    famcmd.wrench.torque.x = T_xyz_B[0];  // 3D only
    famcmd.wrench.torque.y = T_xyz_B[1];  // 3D only
    famcmd.wrench.torque.z = T_xyz_B[2];

    pub_ctl_.publish(famcmd);
  }

  /* ************************************************************************** */
  void CasadiNMPCNodelet::ekf_callback(const ff_msgs::EkfState::ConstPtr msg) {
    /* The `gnc/ekf` subscriber callback
    Called at 62.5 Hz; updates Chaser state.
    x_real_complete_ = [x y z  qx qy qz qw  vx vy vz  wx wy wz  ax ay az]
    */
    // if (use_ekf_dlr_ == true && got_ekf_dlr_ == true) {  // switch over as soon as available
    //   return;
    // }
    // else {
    float qx = msg->pose.orientation.x;
    float qy = msg->pose.orientation.y;
    float qz = msg->pose.orientation.z;
    float qw = msg->pose.orientation.w;
    // check if EKF message is 0 before populating state
    if (state_mode_.compare("ekf") == 0 && (qx != 0 || qy != 0 || qz != 0 || qw != 0) ||
        control_mode_.compare("regulate") == 0) {
      x_real_complete_(0) = msg->pose.position.x;
      x_real_complete_(1) = msg->pose.position.y;
      x_real_complete_(2) = msg->pose.position.z;
      x_real_complete_(3) = msg->pose.orientation.x;
      x_real_complete_(4) = msg->pose.orientation.y;
      x_real_complete_(5) = msg->pose.orientation.z;
      x_real_complete_(6) = msg->pose.orientation.w;
      x_real_complete_(7) = msg->velocity.x;
      x_real_complete_(8) = msg->velocity.y;
      x_real_complete_(9) = msg->velocity.z;
      x_real_complete_(10) = msg->omega.x;
      x_real_complete_(11) = msg->omega.y;
      x_real_complete_(12) = msg->omega.z;
      x_real_complete_(13) = 0.0;
      x_real_complete_(14) = 0.0;
      x_real_complete_(15) = 0.0;

      x_real_(0) = x_real_complete_(0);
      x_real_(1) = x_real_complete_(1);
      x_real_(2) = x_real_complete_(2);
      x_real_(3) = x_real_complete_(7);
      x_real_(4) = x_real_complete_(8);
      x_real_(5) = x_real_complete_(9);
    }
    // }
  }

  /* ************************************************************************** */
  // void CasadiNMPCNodelet::ekf_dlr_callback(const ekf_dlr::EKF_Obs_State::ConstPtr msg) {
  //   /* The `gnc/ekf` subscriber callback
  //   Called at 62.5 Hz; updates Chaser state.
  //   x_real_complete_ = [x y z  qx qy qz qw  vx vy vz  wx wy wz  ax ay az]
  //   */
  //   if (ros::Time::now().toSec() - tic_startup_ > dlr_timeout_period_ && use_ekf_dlr_== true) {  // timeout period complete, use ekf_dlr
  //     if (got_ekf_dlr_ == false){
  //       std::cout << "Swapping to ekf_dlr!" << std::endl;
  //     }
  //     got_ekf_dlr_ = true;

  //     float qx = msg->pose.orientation.x;
  //     float qy = msg->pose.orientation.y;
  //     float qz = msg->pose.orientation.z;
  //     float qw = msg->pose.orientation.w;
  //     // check if EKF message is 0 before populating state
  //     if (state_mode_.compare("ekf") == 0 && (qx != 0 || qy != 0 || qz != 0 || qw != 0) ||
  //         control_mode_.compare("regulate") == 0) {
  //       x_real_complete_(0) = msg->pose.position.x;
  //       x_real_complete_(1) = msg->pose.position.y;
  //       x_real_complete_(2) = msg->pose.position.z;
  //       x_real_complete_(3) = msg->pose.orientation.x;
  //       x_real_complete_(4) = msg->pose.orientation.y;
  //       x_real_complete_(5) = msg->pose.orientation.z;
  //       x_real_complete_(6) = msg->pose.orientation.w;
  //       x_real_complete_(7) = msg->velocity.x;
  //       x_real_complete_(8) = msg->velocity.y;
  //       x_real_complete_(9) = msg->velocity.z;
  //       x_real_complete_(10) = msg->omega.x;
  //       x_real_complete_(11) = msg->omega.y;
  //       x_real_complete_(12) = msg->omega.z;
  //       x_real_complete_(13) = 0.0;
  //       x_real_complete_(14) = 0.0;
  //       x_real_complete_(15) = 0.0;

  //       x_real_(0) = x_real_complete_(0);
  //       x_real_(1) = x_real_complete_(1);
  //       x_real_(2) = x_real_complete_(2);
  //       x_real_(3) = x_real_complete_(7);
  //       x_real_(4) = x_real_complete_(8);
  //       x_real_(5) = x_real_complete_(9);
  //     }
  //   }
  // }

  /* ************************************************************************** */
  void CasadiNMPCNodelet::slam_pose_callback(const geometry_msgs::PoseWithCovariance::ConstPtr msg) {
    /* Update Chaser pose using SLAM. If SLAM spoof is on, this data is from EKF.
    POSITIONS ARE IN TVR FRAME!
    */
    if (state_mode_.compare("slam") == 0 && control_mode_.compare("regulate") != 0) {
      x_real_complete_(0) = msg->pose.position.x + r_RI_(0);  // update positions to ISS frame
      x_real_complete_(1) = msg->pose.position.y + r_RI_(1);
      x_real_complete_(2) = msg->pose.position.z + r_RI_(2);
      x_real_complete_(3) = msg->pose.orientation.x;
      x_real_complete_(4) = msg->pose.orientation.y;
      x_real_complete_(5) = msg->pose.orientation.z;
      x_real_complete_(6) = msg->pose.orientation.w;

      x_real_(0) = x_real_complete_(0);
      x_real_(1) = x_real_complete_(1);
      x_real_(2) = x_real_complete_(2);
    }
  }

  /* ************************************************************************** */
  void CasadiNMPCNodelet::slam_twist_callback(const geometry_msgs::TwistWithCovariance::ConstPtr msg) {
    /* Updates Chaser velocities using SLAM. If SLAM spoof is on, this data is from EKF.
    */
    if (state_mode_.compare("slam") == 0 && control_mode_.compare("regulate") != 0) {
      x_real_complete_(7) = msg->twist.linear.x;
      x_real_complete_(8) = msg->twist.linear.y;
      x_real_complete_(9) = msg->twist.linear.z;
      x_real_complete_(10) = msg->twist.angular.x;
      x_real_complete_(11) = msg->twist.angular.y;
      x_real_complete_(12) = msg->twist.angular.z;
      x_real_complete_(13) = 0.0;
      x_real_complete_(14) = 0.0;
      x_real_complete_(15) = 0.0;

      x_real_(3) = x_real_complete_(7);
      x_real_(4) = x_real_complete_(8);
      x_real_(5) = x_real_complete_(9);
    }
  }

  /* ************************************************************************** */
  void CasadiNMPCNodelet::slam_targ_att_callback(const geometry_msgs::PoseWithCovariance::ConstPtr msg) {
    /* Updates Target pose estimate using SLAM. If SLAM spoof is on, this data is known exactly.
    */
    q_targ_(0) = msg->pose.orientation.x;
    q_targ_(1) = msg->pose.orientation.y;
    q_targ_(2) = msg->pose.orientation.z;
    q_targ_(3) = msg->pose.orientation.w;
  }

  /* ************************************************************************** */
  void CasadiNMPCNodelet::slam_targ_omega_callback(const geometry_msgs::TwistWithCovariance::ConstPtr msg) {
    /* Updates Target angular velocity estimate using SLAM. If SLAM spoof is on, this data is known exactly.
    */
    omega_targ_(0) = msg->twist.angular.x;
    omega_targ_(1) = msg->twist.angular.y;
    omega_targ_(2) = msg->twist.angular.z;
  }
} // end namespace casadi_nmpc