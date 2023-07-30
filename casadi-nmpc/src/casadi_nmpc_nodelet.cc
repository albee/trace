/**
 * @file casadi_nmpc_nodelet.cc
 * @author Keenan Albee
 * @brief Robust tube MPC, nodelet wrapper.
 * 
 * 
 * control_mode_ is the most important parameter that defines what state the controller is in.
 * 
 * MPC can be in mode {track, track_tube, regulate, inactive, debug, unit_test, unit_test_pd}, see Run().
 * Combines translational tube MPC output with PD attitude control provided by backup_controller.
 * 
 * Note: Actuator saturation due to both controllers' computation is currently NOT considered!
 *
 * @version 0.1
 * @date 2023-04-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "casadi_nmpc/casadi_nmpc.h"


namespace casadi_nmpc {
  /* ************************************************************************** */
  void CasadiNMPCNodelet::Initialize(ros::NodeHandle* nh) {
    /* This is called when the nodelet is loaded into the nodelet manager.
    */
    // If in the sim, we need the robot namespace for default topics
    ros::param::getCached("/td/ground", ground_);
    ros::param::getCached("/td/sim", sim_);

    std::string name = ff_util::FreeFlyerNodelet::GetPlatform();

    // load in CasADi functions
    std::string DATA_PATH = ros::package::getPath("data")+"/";
    std::string TUBE_MPC_FILE = DATA_PATH + "input/casadi-functions/" + "tube_mpc_func_casadi.casadi";
    std::string MPC_FILE = DATA_PATH + "input/casadi-functions/" + "mpc_func_casadi.casadi";
    tube_mpc_func_serialized_ = casadi::Function::load(TUBE_MPC_FILE);
    mpc_func_serialized_ = casadi::Function::load(MPC_FILE);
    // mpc_func_casadi_ = casadi::external("mpc_func_casadi", CASADI_MPC_LIB);  // casadi needs to be able to find this .so
    // tube_mpc_func_casadi_ = casadi::external("tube_mpc_func_casadi", CASADI_ROBUST_TUBE_MPC_LIB);  // casadi needs to be able to find this .so

    // Get parameters from config files and from chaser_coordinator
    CasadiNMPCNodelet::get_all_YAML_parameters();
    CasadiNMPCNodelet::get_regulation_and_uc_bound_parameters();

    // Non-parameter initializations
    x_des_traj_N_.resize(N+1, 6);
    u_opt_.setZero(6, 1);
    w_bound_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    x_real_ << 10.9, -9.65, 4.9, 0.0, 0.0, 0.0;  // dummy data until estimator publishes
    x_real_complete_ << 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0; // dummy data until estimator publishes

    // Init subscribers
    sub_ekf_ = nh->subscribe<ff_msgs::EkfState>("gnc/ekf", 5,
      boost::bind(&CasadiNMPCNodelet::ekf_callback, this, _1));  // incoming localization
    // sub_ekf_dlr_ = nh->subscribe<ekf_dlr::EKF_Obs_State>("ekf_observer", 5,
    //   boost::bind(&CasadiNMPCNodelet::ekf_dlr_callback, this, _1));  // incoming localization + filtering ekf on top
    sub_slam_pos_ = nh->subscribe<geometry_msgs::PoseWithCovariance>("/td/mit_slam/chaser_pose", 5,
      boost::bind(&CasadiNMPCNodelet::slam_pose_callback, this, _1));  // incoming chaser position
    sub_slam_vel_ = nh->subscribe<geometry_msgs::TwistWithCovariance>("/td/mit_slam/chaser_twist", 5,
      boost::bind(&CasadiNMPCNodelet::slam_twist_callback, this, _1));  // incoming chaser velocity
    sub_slam_targ_att_ = nh->subscribe<geometry_msgs::PoseWithCovariance>("/td/mit_slam/target_pose", 5,
      boost::bind(&CasadiNMPCNodelet::slam_targ_att_callback, this, _1));
    sub_slam_targ_omega_ = nh->subscribe<geometry_msgs::TwistWithCovariance>("/td/mit_slam/target_twist", 5,
      boost::bind(&CasadiNMPCNodelet::slam_targ_omega_callback, this, _1));
    sub_x_des_traj_ = nh->subscribe<std_msgs::Float64MultiArray>(TOPIC_TD_TUBE_MPC_TRAJ, 5,
      boost::bind(&CasadiNMPCNodelet::x_des_traj_callback, this, _1));  // incoming full traj---used once
    sub_x_des_traj_body_ = nh->subscribe<trace_msgs::TDTrajBody>(TOPIC_TD_TUBE_MPC_TRAJ_BODY, 5,
      boost::bind(&CasadiNMPCNodelet::x_des_traj_body_callback, this, _1));  // incoming full traj---used once
    sub_uc_bound_ = nh->subscribe<std_msgs::Float64MultiArray>(UC_BOUND_TOPIC, 5,
      boost::bind(&CasadiNMPCNodelet::w_bound_callback, this, _1));  // incoming uc_bound
    sub_status_ = nh->subscribe<trace_msgs::TDStatus>(TOPIC_TD_STATUS, 5,
      boost::bind(&CasadiNMPCNodelet::status_callback, this, _1));
    sub_inertia_ = nh->subscribe<geometry_msgs::Inertia>("/td/mit_slam/inertia", 5,
      boost::bind(&CasadiNMPCNodelet::inertia_callback, this, _1));

    // Init pubs
    pub_ctl_ = nh->advertise<ff_msgs::FamCommand>(TOPIC_GNC_CTL_COMMAND, 5, true);  // outgoing FAM commands: /honey/gnc/ctl/command
    pub_debug_ = nh->advertise<trace_msgs::TDCasadiDebug>(TOPIC_TD_TUBE_MPC_DEBUG, 5);      // For timing info and MPC inputs calculated
    pub_eigen_x_des_traj_ = nh->advertise<std_msgs::Float64MultiArray>("td/tube_mpc/traj_updated", 5);      // For timing info and MPC inputs calculated
    pub_casadi_status_ = nh->advertise<trace_msgs::TDCasadiStatus>("td/casadi_nmpc/status", 5, true);  // Status
    pub_mrpi_ = nh->advertise<trace_msgs::TDMRPI_msg>(TOPIC_TD_TUBE_MPC_MRPI, 5, true);  // MRPI

    NODELET_INFO_STREAM("[CASADI_NMPC] Initialized.");
    tic_startup_ = ros::Time::now().toSec(); 

    thread_.reset(new std::thread(&casadi_nmpc::CasadiNMPCNodelet::Run, this));  // hack to get timers to work
  }

  /* ************************************************************************** */
  void CasadiNMPCNodelet::Run() {
    /* ROS spin loop
    */
    ros::NodeHandle MTNH = getMTNodeHandle();  // multithread within nodelet

    /*
    (1) Command Timer (sends out u_opt_)
    sends out commands at command_rate_
    */
    ros::Timer timer_command_rate = MTNH.createTimer(ros::Duration(1/command_rate_),
      boost::bind(&CasadiNMPCNodelet::command_timer_callback, this, _1));  // send commands

    /*
    (2) MPC Request Timer
    request MPC updates at MPC_rate_ (creates u_opt_ based on x_real)
    */
    ros::Timer timer_MPC_rate = MTNH.createTimer(ros::Duration(control_dt_),
      boost::bind(&CasadiNMPCNodelet::MPC_timer_callback, this, _1));  // send commands

    ros::waitForShutdown();
  }

  /* ************************************************************************** */
  void CasadiNMPCNodelet::MPC_timer_callback(const ros::TimerEvent&) {
    /* Main control loop. Run MPC at MPC_rate_.
    Assumes w_bound_ and x_des_traj_ have been set before running.

    control_mode_ for MPC can be: {track, track_tube, regulate, inactive, debug, unit_test, unit_test_pd}
    WAS_TRACK:= {0, 1}, 1 means was previously tracking
    HAVE_TRAJ:= {0, 1}, 1 means a trajectory is loaded in
    */

    // reset trajectory if stopped
    if ((control_mode_.compare("track") != 0 && control_mode_.compare("track_tube") != 0) && WAS_TRACK) {
      WAS_TRACK = 0;
      HAVE_TRAJ = 0;
      traj_idx_ = 0;
    }

    // loop through possible control modes
    if (control_mode_.compare("inactive") == 0) {
      // NODELET_INFO_STREAM("MPC is not active..." << control_mode_);
      u_opt_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    }
    else if (control_mode_.compare("debug") == 0) {
      run_debug();
      control_mode_ = "inactive";
    }
    // Regulate
    else if (control_mode_.compare("regulate") == 0) {
      // NODELET_INFO_STREAM("casadi_nmpc is regulating...");
      update_u_opt_(control_mode_);
    }
    // Standard MPC or Tube MPC
    else if (control_mode_.compare("track") == 0 || control_mode_.compare("track_tube") == 0) {
      auto tic_total = std::chrono::high_resolution_clock::now();
      if (WAS_TRACK == 0) {  // start the time count
        t_start_ = ros::Time::now();
      }
      WAS_TRACK = 1;

      // set regulation to latest state estimate
      Vector3f x0;
      Vector4f a0;
      x0 << x_real_complete_(0), x_real_complete_(1), x_real_complete_(2);  // position
      a0 << x_real_complete_(3), x_real_complete_(4), x_real_complete_(5), x_real_complete_(6);  // attitude, qx qy qz qw
      update_regulation_setpoint(x0, a0);  // update regulation in case we swap over

      if (HAVE_TRAJ != 1) {  // do we actually have a trajectory?
        NODELET_INFO_STREAM("Trajectory has not been set!");
        control_mode_ = "inactive";
      }
      else if (traj_idx_ >= N_traj_) {  // are we finished with the trajectory?
        NODELET_INFO_STREAM("Trajectory complete! Swapping to regulation.");
        // set regulation to final point of traj
        control_mode_ = "regulate";
        traj_finished_ = true;
      }
      else {  // use MPC or Tube MPC
        // Update inertial frame trajectory if getting target attitude estimates
        if (online_update_mode_ == true) {
            CasadiNMPCNodelet::update_inertial_traj();
        }
        update_u_opt_(control_mode_);
        auto toc_total = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> tictoc = toc_total - tic_total;
        total_comp_time_ = tictoc.count();
        NODELET_INFO_STREAM("total_comp_time_: " << total_comp_time_);
      }
    }

    // update /td/casadi_status once every loop
    publish_casadi_status();
  }

  /* ************************************************************************** */
  void CasadiNMPCNodelet::command_timer_callback(const ros::TimerEvent&) {
    /* Publish the latest available u_opt_
    */
    // NODELET_INFO_STREAM("Command callback..." << u_opt_[0] << " " << u_opt_[1] << " " << u_opt_[2] );

    if (control_mode_.compare("inactive") != 0) {
      // Convert to body frame on every call
      tf2::Vector3 F_xyz_I(u_opt_[0], u_opt_[1], u_opt_[2]);  // must convert to Body frame
      tf2::Vector3 T_xyz_B(u_opt_[3], u_opt_[4], u_opt_[5]);

      tf2::Quaternion q_BI;  // convert Inertial to Body
      tf2::Quaternion q_IB(x_real_complete_(3), x_real_complete_(4), x_real_complete_(5), x_real_complete_(6));  // Body wrt Inertial
      q_BI = q_IB.inverse();
      q_BI.normalize();
      tf2::Vector3 F_xyz_B = tf2::Transform(q_BI)*F_xyz_I;

      std::tie(F_xyz_B, T_xyz_B) = check_input_limits(F_xyz_B, T_xyz_B);  // sanity check on input limits---should already be accomplished

      CasadiNMPCNodelet::publish_ctl(F_xyz_B, T_xyz_B);
    }
  }

  /* ************************************************************************** */
  std::tuple <tf2::Vector3, tf2::Vector3> CasadiNMPCNodelet::check_input_limits(tf2::Vector3 F_xyz_B, tf2::Vector3 T_xyz_B) {
    /* Sanity check on commanded forces/torques. These constraints should already be enforced,
    but this step is a second check before commanded the FAM.

    F_xyz_B: force in body frame
    T_xyz_B: torque in body frame
    */
    // force check
    double F_TOL = 0.0;  // no tolerance

    for (int i = 0; i < 3; i++) {
      if (F_xyz_B[i] > (u_mag_(i) + F_TOL)) {
        F_xyz_B[i] = u_mag_(i);
      }
      else if (F_xyz_B[i] < (-u_mag_(i) - F_TOL)) {
        F_xyz_B[i] = -u_mag_(i);
      }
    }

    // torque check
    for (int i = 0; i < 3; i++) {
      if (T_xyz_B[i] > torque_mag_(i)) {
        T_xyz_B[i] = torque_mag_(i);
      }
      else if (T_xyz_B[i] < -torque_mag_(i))  {
        T_xyz_B[i] = -torque_mag_(i);
      }
    }

    return std::make_tuple(F_xyz_B, T_xyz_B);
  }

  /* ************************************************************************** */
  void CasadiNMPCNodelet::update_u_opt_(std::string control_mode_) {
    /* Update u_opt_ with both CasADi translation and PD attitude control.
    */
    Vector3d u_forces;
    Vector3d u_torques;

    Vector3d u0_mpc = Vector3d::Zero();
    Vector3d u0_dr = Vector3d::Zero();
    Matrix<double, 6, 1> x_nom = Matrix<double, 6, 1>::Zero();

    // Tube MPC
    if (control_mode_.compare("track_tube") == 0) {
      std::tie(u_forces, u0_mpc, u0_dr, x_nom) = call_tube_mpc_func_casadi();
      u_torques = calc_torques_PD(false);
      u_opt_ << u_forces, u_torques;

      NODELET_INFO_STREAM(
         "\n******************\n"
      << "TUBE MPC:"
      << "\ntraj_idx_: " << traj_idx_
      << "\nr_des: " << eigen_x_des_traj_.block(traj_idx_, 1, 1, 3)
      << "\nr_real: " << x_real_complete_.segment(0, 3).transpose()
      << "\nquat_des: " << eigen_x_des_traj_.block(traj_idx_, 7, 1, 4)
      << "\nquat_real: " << x_real_complete_.segment(3, 4).transpose()
      << "\nw_des: " << eigen_x_des_traj_.block(traj_idx_, 11, 1, 3)
      << "\nw_real: " << x_real_complete_.segment(10, 3).transpose()
      << "\nu_forces: " << u_forces.transpose()
      << "\nu_torques: " << u_torques.transpose()
      << "\n******************");
    }

    // Standard MPC
    else if (control_mode_.compare("track") == 0){
      u_forces = call_mpc_func_casadi(false);
      u_torques = calc_torques_PD(false);
      u_opt_ << u_forces, u_torques;

    //   NODELET_INFO_STREAM(
    //      "\n******************\n"
    //   << "STANDARD MPC:"
    //   << "\ntraj_idx_: " << traj_idx_
    //   << "\nr_des: " << eigen_x_des_traj_.block(traj_idx_, 1, 1, 3)
    //   << "\nr_real: " << x_real_complete_.segment(0, 3).transpose()
    //   << "\nquat_des: " << eigen_x_des_traj_.block(traj_idx_, 7, 1, 4)
    //   << "\nquat_real: " << x_real_complete_.segment(3, 4).transpose()
    //   << "\nw_des: " << eigen_x_des_traj_.block(traj_idx_, 11, 1, 3)
    //   << "\nw_real: " << x_real_complete_.segment(10, 3).transpose()
    //   << "\nu_forces: " << u_forces.transpose()
    //   << "\nu_torques: " << u_torques.transpose()
    //   << "\n******************");
    }

    // Regulation (using Standard MPC)
    else if (control_mode_.compare("regulate") == 0) {
      u_forces = call_mpc_func_casadi(true);
      u_torques = calc_torques_PD(true);
      u_opt_ << u_forces, u_torques;

      // NODELET_INFO_STREAM(
      //       "\n******************\n"
      //   << "traj_idx_: " << traj_idx_ << " N_traj_: " << N_traj_ << " t_elapsed_: " << t_elapsed_
      //   << "\nquat_des: " << eigen_x_des_traj_reg_.block(traj_idx_, 7, 1, 4)
      //   << "\nquat_real: " << x_real_complete_.segment(3, 4).transpose()
      //   << "\nw_des: " << eigen_x_des_traj_reg_.block(traj_idx_, 11, 1, 3)
      //   << "\nw_real: " << x_real_complete_.segment(10, 3).transpose()
      //   << "\nu_forces: " << u_forces.transpose()
      //   << "\nu_torques: " << u_torques.transpose()
      //   << "\n******************");
    }

    // note time, increment traj_idx_
    t_elapsed_ = ros::Time::now().toSec() - t_start_.toSec();
    if (control_mode_.compare("regulate") != 0) {
      traj_idx_ += 1;  // don't increment traj_idx_ if we're regulating
      // NODELET_INFO_STREAM("timing: " << t_elapsed_ << " " << traj_idx_*control_dt_ << " " << traj_idx_);
      if (casadi_comp_time_ > control_dt_) {
        select_closest_setpoint();  // select closest setpoint rather than use delayed one
      }
    }

    // publish post-processing info if we're tracking or regulating
    if (control_mode_.compare("track") == 0 || control_mode_.compare("track_tube") == 0 ||
      control_mode_.compare("regulate") == 0) {
      publish_debug(u_opt_, u0_mpc, u0_dr, x_nom);  // to get info at MPC rate
      publish_eigen_x_des_traj();
    }
  }

  /* ************************************************************************** */
  void CasadiNMPCNodelet::select_closest_setpoint() {
    /* Select the closest setpoint in the event traj_idx is not updated on time.
    */
    traj_idx_ = round(t_elapsed_/control_dt_) - 1;
    NODELET_INFO_STREAM("new traj_idx: " << traj_idx_);
  }

  /* ************************************************************************** */
  Vector3d CasadiNMPCNodelet::calc_torques_PD(bool regulate) {
    /* Calc torques from Ian's PD controller

    Inputs:
    regulate - true or false

    x_real_complete_ - full state, set globally from Ekf data
    eigen_x_des_traj_ - current trajectory being tracked (x_des taken)
    */
    Vector3d u_torques;

    ff_msgs::EkfState state;
    geometry_msgs::Pose pose;
    geometry_msgs::Point r;
    geometry_msgs::Quaternion q;
    geometry_msgs::Vector3 v;
    geometry_msgs::Vector3 w;

    // x_real_complete_ = [x y z qx qy qz qw vx vy vz wx wyz wz]
    r.x = x_real_complete_(0);
    r.y = x_real_complete_(1);
    r.z = x_real_complete_(2);
    q.x = x_real_complete_(3);
    q.y = x_real_complete_(4);
    q.z = x_real_complete_(5);
    q.w = x_real_complete_(6);
    v.x = x_real_complete_(7);
    v.y = x_real_complete_(8);
    v.z = x_real_complete_(9);
    w.x = x_real_complete_(10);
    w.y = x_real_complete_(11);
    w.z = x_real_complete_(12);
    pose.position = r;
    pose.orientation = q;
    state.pose = pose;
    state.velocity = v;
    state.omega = w;

    // eigen_x_des_traj_ needs to be converted to a ControlState from
    // eigen_x_des_traj_ = [[t x y z xd yd zd qx qy qz qw wx wy wz xdd ydd zdd wxd wyd wzd] ... ]
    if (regulate) {
      r.x = eigen_x_des_traj_reg_(traj_idx_, 1);
      r.y = eigen_x_des_traj_reg_(traj_idx_, 2);
      r.z = eigen_x_des_traj_reg_(traj_idx_, 3);
      v.x = eigen_x_des_traj_reg_(traj_idx_, 4);
      v.y = eigen_x_des_traj_reg_(traj_idx_, 5);
      v.z = eigen_x_des_traj_reg_(traj_idx_, 6);
      q.x = eigen_x_des_traj_reg_(traj_idx_, 7);
      q.y = eigen_x_des_traj_reg_(traj_idx_, 8);
      q.z = eigen_x_des_traj_reg_(traj_idx_, 9);
      q.w = eigen_x_des_traj_reg_(traj_idx_, 10);
      w.x = eigen_x_des_traj_reg_(traj_idx_, 11);
      w.y = eigen_x_des_traj_reg_(traj_idx_, 12);
      w.z = eigen_x_des_traj_reg_(traj_idx_, 13);
    }
    else {
      r.x = eigen_x_des_traj_(traj_idx_, 1);
      r.y = eigen_x_des_traj_(traj_idx_, 2);
      r.z = eigen_x_des_traj_(traj_idx_, 3);
      v.x = eigen_x_des_traj_(traj_idx_, 4);
      v.y = eigen_x_des_traj_(traj_idx_, 5);
      v.z = eigen_x_des_traj_(traj_idx_, 6);
      q.x = eigen_x_des_traj_(traj_idx_, 7);
      q.y = eigen_x_des_traj_(traj_idx_, 8);
      q.z = eigen_x_des_traj_(traj_idx_, 9);
      q.w = eigen_x_des_traj_(traj_idx_, 10);
      w.x = eigen_x_des_traj_(traj_idx_, 11);
      w.y = eigen_x_des_traj_(traj_idx_, 12);
      w.z = eigen_x_des_traj_(traj_idx_, 13);
    }

    ff_msgs::ControlState x_des;
    geometry_msgs::Twist twist;
    pose.position = r;
    pose.orientation = q;
    twist.linear = v;
    twist.angular = w;
    x_des.pose = pose;
    x_des.twist = twist;

    u_torques = pd_control_.controller_main(state, x_des, torque_mag_);  // also performs torque vector scaling
    return u_torques;
  }

  /* ************************************************************************** */
  void CasadiNMPCNodelet::update_inertial_traj() {
    /* Update inertial frame trajectory based on updated target attitude estimates.
    To be as efficient as possible, only update the current nominal trajectory + the next N steps.
    */
    Eigen::Matrix3d R_targ_IB = q2dcm(q_targ_).cast<double>();
    Eigen::Vector3d omega_targ_B = omega_targ_.cast<double>();

    if (ground_.compare("true") == 0) {
      eigen_x_des_traj_(traj_idx_, 3) = -0.7;
      eigen_x_des_traj_(traj_idx_, 6) = 0.0;
    }

    double t = 0.0;

    // attitude updating info
    tf2::Quaternion q_targ_IB_0;
    tf2::Quaternion q_targ_IB_new{q_targ_(0), q_targ_(1), q_targ_(2), q_targ_(3)};  // target body pose wrt inertial frame (updated)
    tf2::Quaternion q_shift;
    Eigen::Matrix3d R_shift;
    tf2::Quaternion q_chaser_IB_0;  // original chaser orientation
    tf2::Quaternion q_chaser_IB_new;  // updated chaser orientation

    // we have these initial conditions from SLAM updates
    state_type x;
    x.segment(0,4) = q_targ_;
    x.segment(4,7) = omega_targ_;

    // NODELET_ERROR_STREAM("casadi q_targ_ is: " << q_targ_);
    // NODELET_ERROR_STREAM("J_vec_\n" << J_vec_ << "\ntarg_offset_\n" << targ_offset_);

    // Go through all remaining setpoints and rotate them into the new predicted Target body frame.
    for (int i = 0; i < (N_traj_ - traj_idx_); i++) {
      // find the relative rotation quaternion from q0 to q_updated (Target)...
      q_targ_IB_0 = tf2::Quaternion{eigen_q_targ_0_hist_(traj_idx_ + i, 0), eigen_q_targ_0_hist_(traj_idx_ + i, 1), eigen_q_targ_0_hist_(traj_idx_ + i, 2), eigen_q_targ_0_hist_(traj_idx_ + i, 3)};
      q_shift = q_targ_IB_new * q_targ_IB_0.inverse();
      R_shift = q2dcm(Vector4f(static_cast<float>(q_shift[0]), static_cast<float>(q_shift[1]),
                               static_cast<float>(q_shift[2]), static_cast<float>(q_shift[3]))).cast<double>();
      // ...and apply this rotation to the planned Chaser attitude.

      // if (i == 0 || i == 1) {
      //   NODELET_ERROR_STREAM("R_shift: \n" << R_shift);
      //   NODELET_ERROR_STREAM("q_targ_IB_0: \n" << quat2str(q_targ_IB_0));
      // }

      // Compute position in inertial frame
      eigen_x_des_traj_.block(traj_idx_ + i, 1, 1, 3) = (R_targ_IB * eigen_x_des_traj_body_.block(traj_idx_ + i, 1, 1, 3).transpose() + targ_offset_).transpose();

      // Compute velocity in inertial frame (use Coriolis theorem)
      // R_rot2ine(v_rot + w_rod/ine x r_rot)
      Eigen::Vector3d pos_des_B = eigen_x_des_traj_body_.block(traj_idx_ + i, 1, 1, 3).transpose(); // body frame
      eigen_x_des_traj_.block(traj_idx_ + i, 4, 1, 3) = (R_targ_IB * (eigen_x_des_traj_body_.block(traj_idx_ + i, 4, 1, 3).transpose() + omega_targ_B.cross(pos_des_B)) ).transpose();

      // Compute shifted q_chaser (body traj has quat info so let's use it---never changes)
      q_chaser_IB_0 = tf2::Quaternion{eigen_x_des_traj_body_(traj_idx_ + i, 7), eigen_x_des_traj_body_(traj_idx_ + i, 8), eigen_x_des_traj_body_(traj_idx_ + i, 9), eigen_x_des_traj_body_(traj_idx_ + i, 10)};
      q_chaser_IB_new = q_shift*q_chaser_IB_0;
      // Eigen::Vector4d eigen_q_chaser_IB_new
      // q_chaser_IB_new[0], q_chaser_IB_new[1], q_chaser_IB_new[2], q_chaser_IB_new[3]};
      eigen_x_des_traj_.block(traj_idx_ + i, 7, 1, 4) << q_chaser_IB_new[0], q_chaser_IB_new[1], q_chaser_IB_new[2], q_chaser_IB_new[3];

      // Compute shifted omega_B
      eigen_x_des_traj_.block(traj_idx_ + i, 11, 1, 3) = R_shift*eigen_x_des_traj_body_.block(traj_idx_ + i, 11, 1, 3);

      // NODELET_ERROR_STREAM("q_targ_IB_0: " << quat2str(q_targ_IB_0) << " q_targ_IB_new: " << quat2str(q_targ_IB_new) << "q_shift: " << quat2str(q_shift) <<
      // "q_chaser_IB_0: " << quat2str(q_chaser_IB_0) << "q_chaser_IB_new: " << quat2str(q_chaser_IB_new) << "\nq_chaser_w_0: " << eigen_x_des_traj_body_.block(traj_idx_ + i, 11, 1, 3) <<
      // "q_chaser_w_new: " << eigen_x_des_traj_.block(traj_idx_ + i, 11, 1, 3));

      // NODELET_ERROR_STREAM("quat:\n" << eigen_x_des_traj_.block(traj_idx_ + i, 7, 1, 4)
      //   << "\nang vel:\n" << eigen_x_des_traj_.block(traj_idx_ + i, 11, 1, 3));

      if (ground_.compare("true") == 0) {
          eigen_x_des_traj_(traj_idx_ + i, 3) = -0.7;
          eigen_x_des_traj_(traj_idx_ + i, 6) = 0.0;
      }

      // dynamic step the Target---get orientation, R_targ_IB
      double t_next = t + dt_traj_;

      if (ground_.compare("true") == 0) {
        integrate_adaptive(stepper_, dynamic_step_ground_{J_targ_}, x, t, t_next, dt_traj_);
      }
      else {
        integrate_adaptive(stepper_, dynamic_step_{J_targ_}, x, t, t_next, dt_traj_);
      }

      t += dt_traj_;
      R_targ_IB = q2dcm(x.head(4)).cast<double>();  // propagated target orientation
      q_targ_IB_new = tf2::Quaternion{x(0), x(1), x(2), x(3)};
      // R_targ_IB << 1, 0, 0, 0, 1, 0, 0, 0, 1;
      omega_targ_B = x.tail(3).transpose().cast<double>();  // target angular velocity (inertial frame)
      // NODELET_ERROR_STREAM("\nR_targ_IB\n" << R_targ_IB << "\nomega_targ_B\n" << omega_targ_B);
    }

    // NODELET_INFO_STREAM(
    //     "\n******************\n"
    //  << "IN UPDATE:"
    //  << "\ntraj_idx_: " << traj_idx_
    //  << "\nr_des: " << eigen_x_des_traj_.block(traj_idx_+1, 1, 1, 3)
    //  << "\nr_real: " << x_real_complete_.segment(0, 3).transpose()
    //  << "\nquat_des: " << eigen_x_des_traj_.block(traj_idx_+1, 7, 1, 4)
    //  << "\nquat_real: " << x_real_complete_.segment(3, 4).transpose()
    //  << "\nw_des: " << eigen_x_des_traj_.block(traj_idx_+1, 11, 1, 3)
    //  << "\nw_real: " << x_real_complete_.segment(10, 3).transpose()
    //  << "\n******************");
  }

  /* ************************************************************************** */
  void CasadiNMPCNodelet::run_debug() {
    NODELET_INFO_STREAM("debug...");
    w_bound_ << 0.0331, 0.0260, 0.0537, 0.0069, 0.0055, 0.0073;  // usually w_bound is set by uc_bound
    x_real_ << 10.85, -9.65, 4.9, 0.0, 0.0, 0.0;

    CasadiNMPCNodelet::read_traj_standard_format("input/TEST12-ISS/");  // read it in ourselves

    traj_idx_ = 0;
    state_mode_ = "ekf";
    HAVE_TRAJ = true;

    // update_inertial_traj();

    // get_setpoints();

    // mpc call test
    unit_test_mpc();

    // unit_test_pd();
  }

  void CasadiNMPCNodelet::unit_test_mpc() {
    CasadiNMPCNodelet::read_traj_standard_format("input/TEST12-ISS/");  // read it in ourselves

    Vector3d u_forces;
    Vector3d u_torques;

    Vector3d u0_mpc;
    Vector3d u0_dr;
    Matrix<double, 6, 1> x_nom = Matrix<double, 6, 1>::Zero();

    t_start_ = ros::Time::now();

    NODELET_INFO_STREAM(
       "\n******************\n"
    << "w_bound_: " << w_bound_
    << "\nu_mag_: " << u_mag_
    << "\nMPC_dt_: " << MPC_dt_
    << "\nmass_: " << mass_
    << "\nQ_pos_anc_factor: " << Q_pos_anc_factor_
    << "\nQ_vel_anc_factor: " << Q_vel_anc_factor_
    << "\nR_anc_factor: " << R_anc_factor_
    << "\nusing_fallback_mrpi: " << using_fallback_mrpi_
    << "\n******************");

    prepare_mrpi(w_bound_, u_mag_, MPC_dt_, mass_, Q_pos_anc_factor_, Q_vel_anc_factor_, R_anc_factor_);  // get K_dr, Au, bu, AZ, bZ ready

    NODELET_INFO_STREAM("calling tube MPC...");
    update_u_opt_("track_tube");

    /// Compute the tube MPC input
    std::tie(u_forces, u0_mpc, u0_dr, x_nom) = call_tube_mpc_func_casadi();
    u_torques = calc_torques_PD(false);
    u_opt_ << u_forces, u_torques;

    NODELET_INFO_STREAM(
       "\n******************\n"
    << "traj_idx_: " << traj_idx_
    << "\nr_des: " << eigen_x_des_traj_.block(traj_idx_, 1, 1, 3)
    << "\nr_real: " << x_real_complete_.segment(0, 3).transpose()
    << "\nquat_des: " << eigen_x_des_traj_.block(traj_idx_, 7, 1, 4)
    << "\nquat_real: " << x_real_complete_.segment(3, 4).transpose()
    << "\nw_des: " << eigen_x_des_traj_.block(traj_idx_, 11, 1, 3)
    << "\nw_real: " << x_real_complete_.segment(10, 3).transpose()
    << "\nu_forces: " << u_forces.transpose()
    << "\nu_torques: " << u_torques.transpose()
    << "\n******************");
  }

  /* ************************************************************************** */
  Vector3d CasadiNMPCNodelet::call_pd_and_print() {
    Vector3d u_torques = calc_torques_PD(false);
    NODELET_INFO_STREAM(
          "\n******************\n"
      << "traj_idx_: " << traj_idx_
      << "\nquat_des: " << eigen_x_des_traj_.block(traj_idx_, 7, 1, 4)
      << "\nquat_real: " << x_real_complete_.segment(3, 4).transpose()
      << "\nw_des: " << eigen_x_des_traj_.block(traj_idx_, 11, 1, 3)
      << "\nw_real: " << x_real_complete_.segment(10, 3).transpose()
      // << "\nu_forces: " << u_forces.transpose()
      << "\nu_torques: " << u_torques.transpose()
      << "\n******************");
      return u_torques;
  }

  /* ************************************************************************** */
  std::tuple<Vector3d, Vector3d, Vector3d, Matrix<double, 6, 1>> CasadiNMPCNodelet::call_tube_mpc_func_casadi(){
    /* Call CasADi.
    Input:
    casadi_args (see below)

    Output:
    tuple of
    u_forces [3x1] forces including ancillary input
    x_nom - [6x1] nominal state (x0)
    */
    Vector3d u_forces;

    // debug info
    Vector3d u0_mpc;
    Vector3d u0_dr;
    Eigen::Matrix<double, 6, 1> x_nom;

    vector<casadi::DM> casadi_args = {};  // input vector to CasADi function

    casadi::DM dm_x0_ = eigen2dm(x_real_);  // latest state estimate
    casadi::DM dm_u_mag = eigen2dm(u_mag_);
    x_des_traj_N_ = get_setpoints();        // get setpoint values
    casadi::DM dm_x_des_traj_N = eigen2dm(x_des_traj_N_);  // ugly Eigen --> DM conversion

    // IOFormat HeavyFmt(FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");
    // std::string sep = "\n----------------------------------------\n";
    // NODELET_INFO_STREAM(sep << eigen_x_des_traj_.block(traj_idx_,0,1,7).format(HeavyFmt) << sep);
    // NODELET_INFO_STREAM(x_des_traj_N_ << "\n");

    // NODELET_INFO_STREAM("\nx0\n" << dm_x0_ << "\nx_des\n" << dm_x_des_traj_N << "\nu_mag\n" << dm_u_mag << "\nm\n"<< dm_m_ << "\nAu\n" << dm_Au_ << "\nbu\n" << dm_bu_ << "\nAz\n"
    // << dm_AZ_ << "\nbZ\n" << dm_bZ_ << "\nK_dr\n" << dm_K_dr_ << "\nQ1\n" << dm_Q1_T_ << "\nQ2\n" << dm_Q2_T_ << "\nQ3\n" << dm_Q3_T_ << "\nQ4\n" << dm_Q4_T_ << "\nQ5\n"
    // << dm_Q5_T_ << "\nQ6\n" << dm_Q6_T_ << "\nR1\n" << dm_R1_T_ << "\nR2\n"<< dm_R2_T_ << "\nR3\n" << dm_R3_T_ << "\nQN1\n" << dm_QN1_T_ << "\nQN2\n" << dm_QN2_T_ << "\nQN3\n"
    // << dm_QN3_T_ << "\nQN4\n" << dm_QN4_T_ << "\nQN5\n" << dm_QN5_T_ << "\nQN6\n" << dm_QN6_T_);

    //--- CasADi arguments
    casadi_args.push_back(dm_x0_); // for lasso constraint
    casadi_args.push_back(dm_x_des_traj_N);
    casadi_args.push_back(dm_u_mag);
    casadi_args.push_back(dm_m_);
    casadi_args.push_back(dm_Au_);
    casadi_args.push_back(dm_bu_);
    casadi_args.push_back(dm_AZ_);
    casadi_args.push_back(dm_bZ_);
    casadi_args.push_back(dm_K_dr_);
    casadi_args.push_back(dm_Q1_T_);
    casadi_args.push_back(dm_Q2_T_);
    casadi_args.push_back(dm_Q3_T_);
    casadi_args.push_back(dm_Q4_T_);
    casadi_args.push_back(dm_Q5_T_);
    casadi_args.push_back(dm_Q6_T_);
    casadi_args.push_back(dm_R1_T_);
    casadi_args.push_back(dm_R2_T_);
    casadi_args.push_back(dm_R3_T_);
    casadi_args.push_back(dm_QN1_T_);
    casadi_args.push_back(dm_QN2_T_);
    casadi_args.push_back(dm_QN3_T_);
    casadi_args.push_back(dm_QN4_T_);
    casadi_args.push_back(dm_QN5_T_);
    casadi_args.push_back(dm_QN6_T_);

    // Note: CasADi C++ interface takes std::vectors of CasADi type arguments! Make sure casadi::DM, casadi::MX, or casadi::SX are used
    // vector{x0, x_des, u_mag, m, Au, bu, AZ, bZ, K_dr, Q1, Q2, Q3, Q4, Q5, Q6, R1, R2, R3, QN1, QN2, QN3, QN4, QN5, QN6}
    // std::vector<casadi::DM> casadi_out = tube_mpc_func_casadi_(casadi_args);  // nominal MPC
    auto tic = std::chrono::high_resolution_clock::now();
    // std::vector<casadi::DM> casadi_out = tube_mpc_func_casadi_(casadi_args);
    std::vector<casadi::DM> casadi_out = tube_mpc_func_serialized_(casadi_args);  // nominal MPC
    auto toc = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> tictoc = toc - tic;
    casadi_comp_time_ = tictoc.count();
    // NODELET_INFO_STREAM("debug " << casadi_comp_time_);

    // grab CasADi inputs
    u_forces(0) = double(casadi_out.at(0)(0));
    u_forces(1) = double(casadi_out.at(0)(1));
    u_forces(2) = double(casadi_out.at(0)(2));
    if (ground_.compare("true") == 0) { // Zero any z-axis control input for ground.
      u_forces(2) = 0.0;
    }

    u0_mpc(0) = double(casadi_out.at(1)(0));
    u0_mpc(1) = double(casadi_out.at(1)(1));
    u0_mpc(2) = double(casadi_out.at(1)(2));

    u0_dr(0) = double(casadi_out.at(2)(0));
    u0_dr(1) = double(casadi_out.at(2)(1));
    u0_dr(2) = double(casadi_out.at(2)(2));

    x_nom(0) = double(casadi_out.at(3)(0));
    x_nom(1) = double(casadi_out.at(3)(1));
    x_nom(2) = double(casadi_out.at(3)(2));
    x_nom(3) = double(casadi_out.at(3)(3));
    x_nom(4) = double(casadi_out.at(3)(4));
    x_nom(5) = double(casadi_out.at(3)(5));

    // NODELET_INFO_STREAM("traj_idx_: " << traj_idx_ << " N_traj_: " << N_traj_ << " t_elapsed_: " << t_elapsed_
    //              << "\nu_forces:\n" << u_forces);

    return std::make_tuple(u_forces, u0_mpc, u0_dr, x_nom);
  }

  /* ************************************************************************** */
  Vector3d CasadiNMPCNodelet::call_mpc_func_casadi(bool regulate){
    /* Call CasADi.
    Input:
    regulate: bool, true if regulating
    casadi_args (see below)

    Output:
    u_forces [3x1]
    */
    Vector3d u_forces;
    vector<casadi::DM> casadi_args = {};  // input vector to CasADi function

    // Gather remaining CasADi args
    casadi::DM dm_x0_ = eigen2dm(x_real_);  // latest state estimate
    casadi::DM dm_x_des_traj_N;
    casadi::DM dm_u_mag = eigen2dm(u_mag_);

    // Set x_des_traj_N based on either regulation or tracking
    if (regulate) {
      NODELET_DEBUG_STREAM("[CASADI]: Des state horizon: \n" << eigen_x_des_traj_reg_.block(traj_idx_, 1, N+1, 6));
      x_des_traj_N_ = eigen_x_des_traj_reg_.block(traj_idx_, 1, N+1, 6);  // equivalent to MATLAB eigen_x_des_traj_(idx:idx+N+1, 0:6);
      dm_x_des_traj_N = eigen2dm(x_des_traj_N_);  // ugly Eigen --> DM conversion
    }
    else {
      if (traj_idx_ + (N+1) < N_traj_) {
        NODELET_DEBUG_STREAM("[CASADI]: Des state horizon: \n" << eigen_x_des_traj_.block(traj_idx_, 1, N+1, 6));
      }
      x_des_traj_N_ = get_setpoints();
      dm_x_des_traj_N = eigen2dm(x_des_traj_N_);  // ugly Eigen --> DM conversion
    }
    NODELET_DEBUG_STREAM("[CASADI]: Real state: \n" << x_real_.transpose());

    //--- CasADi arguments
    casadi_args.push_back(dm_x0_);
    casadi_args.push_back(dm_x_des_traj_N);
    casadi_args.push_back(dm_u_mag);
    casadi_args.push_back(dm_m_);
    casadi_args.push_back(dm_Q1_);
    casadi_args.push_back(dm_Q2_);
    casadi_args.push_back(dm_Q3_);
    casadi_args.push_back(dm_Q4_);
    casadi_args.push_back(dm_Q5_);
    casadi_args.push_back(dm_Q6_);
    casadi_args.push_back(dm_R1_);
    casadi_args.push_back(dm_R2_);
    casadi_args.push_back(dm_R3_);
    casadi_args.push_back(dm_QN1_);
    casadi_args.push_back(dm_QN2_);
    casadi_args.push_back(dm_QN3_);
    casadi_args.push_back(dm_QN4_);
    casadi_args.push_back(dm_QN5_);
    casadi_args.push_back(dm_QN6_);

    // Note: CasADi C++ interface takes std::vectors of CasADi type arguments! Make sure casadi::DM, casadi::MX, or casadi::SX are used
    // vector{x0, x_des_traj_N, u_mag, m, Q1, Q2, Q3, Q4, Q5, Q6, R1, R2, R3, QN1, QN2, QN3, QN4, QN5, QN6}
    auto tic = std::chrono::high_resolution_clock::now();
    // std::vector<casadi::DM> casadi_out = mpc_func_casadi_(casadi_args);
    std::vector<casadi::DM> casadi_out = mpc_func_serialized_(casadi_args);  // nominal MPC
    auto toc = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> tictoc = toc - tic;
    casadi_comp_time_ = tictoc.count();
    NODELET_INFO_STREAM("debug " << casadi_comp_time_);

    // grab CasADi force output
    u_forces(0) = double(casadi_out.at(0)(0));
    u_forces(1) = double(casadi_out.at(0)(1));
    u_forces(2) = double(casadi_out.at(0)(2));

    // Zero any z-axis control input for ground.
    if (ground_.compare("true") == 0) {
      u_forces(2) = 0.0;
    }

    // NODELET_INFO_STREAM("\ntraj_idx_: " << traj_idx_ << " N_traj_: " << N_traj_ << " t_elapsed_: " << t_elapsed_
    //              << "\nu_forces:\n" << u_forces);

    return u_forces;
  }

  /* ************************************************************************** */
  Eigen::MatrixXd CasadiNMPCNodelet::get_setpoints() {
    /* Return N+1 setpoints from x_des_traj_. If traj_idx_ > N_traj_ - N, then
    start repeating the last setpoint. Account for setpoint stepping.
    */
    int setpoint_step = round(MPC_dt_/dt_traj_);  // should be a nice int: the amount that we skip forward
    int setpoints_required = setpoint_step*N+1;  // includes current time
    NODELET_INFO_STREAM("setpoint checks: " << setpoint_step << " " << setpoints_required << " " << traj_idx_ << " " << N_traj_);

    Eigen::MatrixXd x_des_traj_N = Eigen::MatrixXd::Zero(N+1, 6);

    if (traj_idx_ + setpoints_required - 1 < N_traj_) {  // if there are enough setpoints, use them all
      for (int i = 0; i <= N; i++) {
        NODELET_INFO_STREAM("idx : " << traj_idx_+i*setpoint_step);
        x_des_traj_N.row(i) = eigen_x_des_traj_.block(traj_idx_+i*setpoint_step, 1, 1, 6);  // equivalent to MATLAB eigen_x_des_traj_(idx:idx+N+1, 0:6);
      }
      NODELET_INFO_STREAM("x_des_traj_N: " << x_des_traj_N);
    }
    else {
      int setpoints_available = ((N_traj_ - 1) - traj_idx_) + 1;  // includes current time
      int N_available = floor((setpoints_available - 1)/setpoint_step);  // setpoint steps forward available
      int N_rep = N+1 - (N_available); // number of repeated regulation points
      // x_des_traj_N.block(0, 0, N_available, 6) = eigen_x_des_traj_.block(traj_idx_, 1, N_available, 6);

      // available setpoints
      for (int i = 0; i <= N_available; i++) {
        NODELET_INFO_STREAM("idx : " << traj_idx_+i*setpoint_step);
        x_des_traj_N.row(i) = eigen_x_des_traj_.block(traj_idx_+i*setpoint_step, 1, 1, 6);  // equivalent to MATLAB eigen_x_des_traj_(idx:idx+N+1, 0:6);
      }
      // NODELET_INFO_STREAM("x_des_traj_N (w/ regulate), before update: " << x_des_traj_N);

      // regulation setpoints
      MatrixXd last_row = eigen_x_des_traj_.block(eigen_x_des_traj_.rows()-1, 1, 1, 6);
      x_des_traj_N.block(N_available, 0, N_rep, 6) = last_row.replicate(N_rep, 1);

      NODELET_INFO_STREAM("setpoint end checks: " << setpoints_available << " " << N_rep << " " << N_available << " "
        << " " << last_row.replicate(N_rep, 1));

      NODELET_INFO_STREAM("x_des_traj_N (w/ regulate), after update: " << x_des_traj_N);
    }

    return x_des_traj_N;
  }

  ///
  /// Support functions
  ///
  /* ************************************************************************** */
  void CasadiNMPCNodelet::prepare_mrpi(Eigen::MatrixXd w, Eigen::MatrixXd u_max, double dt, double mass, double Q_pos_anc, double Q_vel_anc, double R_anc){
    /* Call MRPI calc via ROS service to get K_dr and Au and bu. Use these for CasADi update.
    Inputs:
    w - [6x1]
    u_max [3x1]
    dt - scalar
    mass_
    Q_pos_anc
    Q_vel_anc
    R_anc

    Outputs:
    K_
    Au_
    bu_
    AZ_
    bZ_
    */

    // w, u_max, and dt to ROS msg format
    trace_msgs::TDMRPI_srv srv;  // contains .request and .response

    std_msgs::Float64MultiArray w_msg;
    tf::matrixEigenToMsg(w, w_msg);  // Eigen --> msg
    srv.request.w = w_msg;

    std_msgs::Float64MultiArray u_max_msg;
    tf::matrixEigenToMsg(u_max, u_max_msg);  // Eigen --> msg
    srv.request.u_max = u_max_msg;

    srv.request.dt = dt;
    srv.request.mass = mass;
    srv.request.Q_pos_anc = Q_pos_anc;
    srv.request.Q_vel_anc = Q_vel_anc;
    srv.request.R_anc = R_anc;

    // bare call to service
    if (ros::service::call("mrpi", srv))
    {
      NODELET_INFO_STREAM("mrpi service request sent...");
      trace_msgs::TDMRPI_srv::Response res = srv.response;

      // std::vector to Eigen
      // Stride is used for row-major: eigen defaults to column major but numpy uses row major!
      int i = res.K.layout.dim[0].size;
      int j = res.K.layout.dim[1].size;
      K_dr_ = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(res.K.data.data(), i, j);
      dm_K_dr_ = eigen2dm(K_dr_);

      // std::vector to Eigen
      i = res.Au.layout.dim[0].size;
      j = res.Au.layout.dim[1].size;
      Au_ = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(res.Au.data.data(), i, j);
      dm_Au_ = eigen2dm(Au_);

      // std::vector to Eigen
      i = res.bu.layout.dim[0].size;
      j = res.bu.layout.dim[1].size;
      bu_ = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(res.bu.data.data(), i, j);
      dm_bu_ = eigen2dm(bu_);

      i = res.AZ.layout.dim[0].size;
      j = res.AZ.layout.dim[1].size;
      AZ_ = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(res.AZ.data.data(), i, j);
      dm_AZ_ = eigen2dm(AZ_);

      i = res.bZ.layout.dim[0].size;
      j = res.bZ.layout.dim[1].size;
      bZ_ = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(res.bZ.data.data(), i, j);
      dm_bZ_ = eigen2dm(bZ_);

      NODELET_INFO_STREAM("...mrpi service call success!");
    }
    else {  // service call failed, use hard-coded values
      set_mRPI_fallback_values();
      NODELET_INFO_STREAM("...mrpi service unavailable!");
    }

    publish_mrpi();
    mrpi_finished_ = true;
  }

  /* ************************************************************************** */
  void CasadiNMPCNodelet::read_traj_standard_format(std::string traj_filename){
    /*
    Read in the trajectory directly (Caroline format), no passing through ROS
    */
    std::string DATA_PATH = ros::package::getPath("data")+"/";
    std::string traj_file = DATA_PATH + traj_filename;

    // Set very important variables
    eigen_x_des_traj_ = traj_utils::get_planner_output_x<MatrixXd>(traj_file);  // read in .dat trajectory
    dt_traj_ = eigen_x_des_traj_(1, 0) - eigen_x_des_traj_(0, 0);
    traj_rate_ = 1.0/dt_traj_;
    HAVE_TRAJ = 1;
    N_traj_ = eigen_x_des_traj_.rows();
  }
}  // end namespace casadi_nmpc

// Declare the plugin
PLUGINLIB_EXPORT_CLASS(casadi_nmpc::CasadiNMPCNodelet, nodelet::Nodelet);