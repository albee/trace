/**
 * @file casadi_nmpc_utils.cc
 * @author Keenan Albee
 * @brief Utility functions for casadi_nmpc.
 * @version 0.1
 * @date 2023-04-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "casadi_nmpc/casadi_nmpc.h"


namespace casadi_nmpc{
  /* ************************************************************************** */
  void CasadiNMPCNodelet::get_regulation_and_uc_bound_parameters() {
    /* Get parameters specifying where to start the regulation
    */
    if (!casadi_on_target_) {
      ros::param::getCached("/td/chaser/x_start", x0_(0));
      ros::param::getCached("/td/chaser/y_start", x0_(1));
      ros::param::getCached("/td/chaser/z_start", x0_(2));
      ros::param::getCached("/td/chaser/qx_start", a0_(0));
      ros::param::getCached("/td/chaser/qy_start", a0_(1));
      ros::param::getCached("/td/chaser/qz_start", a0_(2));
      ros::param::getCached("/td/chaser/qw_start", a0_(3));
    }
    else {
      ros::param::getCached("/td/target/x_start", x0_(0));
      ros::param::getCached("/td/target/y_start", x0_(1));
      ros::param::getCached("/td/target/z_start", x0_(2));
      ros::param::getCached("/td/target/qx_start", a0_(0));
      ros::param::getCached("/td/target/qy_start", a0_(1));
      ros::param::getCached("/td/target/qz_start", a0_(2));
      ros::param::getCached("/td/target/qw_start", a0_(3));
    }

    // target pos IC (INERTIAL frame)
    ros::param::getCached("/td/chaser/targ_offset_x", targ_offset_(0));
    ros::param::getCached("/td/chaser/targ_offset_y", targ_offset_(1));
    ros::param::getCached("/td/chaser/targ_offset_z", targ_offset_(2));

    update_regulation_setpoint(x0_, a0_);
  }

  /* ************************************************************************** */
  void CasadiNMPCNodelet::update_regulation_setpoint(Vector3f x0, Vector4f a0) {
    /* Update eigen_x_des_traj_reg_ using x0 [x y z] and a0 [qx qy qz qw]
    */
    for (int i = 0; i < eigen_x_des_traj_reg_.rows(); i++) {
      eigen_x_des_traj_reg_(i,1) = (double)x0(0);
      eigen_x_des_traj_reg_(i,2) = (double)x0(1);
      eigen_x_des_traj_reg_(i,3) = (double)x0(2);
      eigen_x_des_traj_reg_(i,4) = 0;
      eigen_x_des_traj_reg_(i,5) = 0;
      eigen_x_des_traj_reg_(i,6) = 0;
      eigen_x_des_traj_reg_(i,7) = (double)a0(0);
      eigen_x_des_traj_reg_(i,8) = (double)a0(1);
      eigen_x_des_traj_reg_(i,9) = (double)a0(2);
      eigen_x_des_traj_reg_(i,10) = (double)a0(3);
      eigen_x_des_traj_reg_(i,11) = 0;
      eigen_x_des_traj_reg_(i,12) = 0;
      eigen_x_des_traj_reg_(i,13) = 0;
    }
  }

  /* ************************************************************************** */
  void CasadiNMPCNodelet::get_all_YAML_parameters() {
    /* Get all parameters needed for MPC and tube MPC. Most are specified by YAML.
    */
    // TVR frame
    ros::param::getCached("/td/r_RI_ISS_x", r_RI_(0));
    ros::param::getCached("/td/r_RI_ISS_y", r_RI_(1));
    ros::param::getCached("/td/r_RI_ISS_z", r_RI_(2));

    double u_mag;
    double torque_mag;

    ros::param::getCached("/td/casadi_nmpc/control_period", control_dt_);
    ros::param::getCached("/td/casadi_nmpc/T", T);
    ros::param::getCached("/td/casadi_nmpc/N", N);
    ros::param::getCached("/td/casadi_nmpc/u_mag", u_mag);
    ros::param::getCached("/td/casadi_nmpc/torque_mag", torque_mag);
    MPC_rate_ = (double)N/T;
    MPC_dt_ = 1.0/MPC_rate_;
    u_mag_ << u_mag, u_mag, u_mag;
    torque_mag_ << torque_mag, torque_mag, torque_mag;

    // ros::param::getCached("/td/use_ekf_dlr", use_ekf_dlr_);
    // ros::param::getCached("/td/dlr_timeout_period", dlr_timeout_period_);
    
    // set gains and mass
    gains gains;
    if (ground_.compare("true") == 0) {
      ros::param::getCached("/td/casadi_nmpc/Q_pos_factor_ground", gains.Q_pos_factor);
      ros::param::getCached("/td/casadi_nmpc/Q_vel_factor_ground", gains.Q_vel_factor);
      ros::param::getCached("/td/casadi_nmpc/R_factor_ground", gains.R_factor);
      ros::param::getCached("/td/casadi_nmpc/QN_pos_factor_ground", gains.QN_pos_factor);
      ros::param::getCached("/td/casadi_nmpc/QN_vel_factor_ground", gains.QN_vel_factor);

      ros::param::getCached("/td/casadi_nmpc/Q_pos_tube_factor_ground", gains.Q_pos_tube_factor);
      ros::param::getCached("/td/casadi_nmpc/Q_vel_tube_factor_ground", gains.Q_vel_tube_factor);
      ros::param::getCached("/td/casadi_nmpc/R_tube_factor_ground", gains.R_tube_factor);
      ros::param::getCached("/td/casadi_nmpc/QN_pos_tube_factor_ground", gains.QN_pos_tube_factor);
      ros::param::getCached("/td/casadi_nmpc/QN_vel_tube_factor_ground", gains.QN_vel_tube_factor);

      ros::param::getCached("/td/casadi_nmpc/mass_ground", mass_);
    }
    else {
      ros::param::getCached("/td/casadi_nmpc/Q_pos_factor_iss", gains.Q_pos_factor);
      ros::param::getCached("/td/casadi_nmpc/Q_vel_factor_iss", gains.Q_vel_factor);
      ros::param::getCached("/td/casadi_nmpc/R_factor_iss", gains.R_factor);
      ros::param::getCached("/td/casadi_nmpc/QN_pos_factor_iss", gains.QN_pos_factor);
      ros::param::getCached("/td/casadi_nmpc/QN_vel_factor_iss", gains.QN_vel_factor);

      ros::param::getCached("/td/casadi_nmpc/Q_pos_tube_factor_iss", gains.Q_pos_tube_factor);
      ros::param::getCached("/td/casadi_nmpc/Q_vel_tube_factor_iss", gains.Q_vel_tube_factor);
      ros::param::getCached("/td/casadi_nmpc/R_tube_factor_iss", gains.R_tube_factor);
      ros::param::getCached("/td/casadi_nmpc/QN_pos_tube_factor_iss", gains.QN_pos_tube_factor);
      ros::param::getCached("/td/casadi_nmpc/QN_vel_tube_factor_iss", gains.QN_vel_tube_factor);

      ros::param::getCached("/td/casadi_nmpc/mass_iss", mass_);
    }

    ros::param::getCached("/td/casadi_nmpc/Q_pos_anc", gains.Q_pos_anc_factor);
    ros::param::getCached("/td/casadi_nmpc/Q_vel_anc", gains.Q_vel_anc_factor);
    ros::param::getCached("/td/casadi_nmpc/R_anc", gains.R_anc_factor);

    make_gains(gains);  // create all gain values
    switch_gains(0);
    dm_m_ = casadi::DM{mass_};  // use casadi uniform constructor

    // if (!casadi_on_target_) {  // Target inertia tensor
    //   ros::param::getCached("/td/uc_bound/target_J", J_vec_);
    //   int k = 0;
    //   for (int i = 0; i < 3; i++) {
    //     for (int j = 0; j < 3; j++) {
    //         J_targ_(i,j) = J_vec_[k];
    //         k++;
    //     }
    //   }
    // }
  }

  /* ************************************************************************** */
  std::string CasadiNMPCNodelet::quat2str(tf2::Quaternion q) {
    std::stringstream ss;
    ss << q[0] << "," << q[1] << "," << q[2] << "," << q[3];
    return ss.str();
  }

  /* ************************************************************************** */
  Eigen::Matrix3f CasadiNMPCNodelet::q2dcm(const Vector4f &q) {
    /* Quaternion (x, y, z, w) to direction cosine matrix

    Gehring, C., Bellicoso, C. D., Bloesch, M., Sommer, H., Fankhauser, P., Hutter, M., & Siegwart, R. (n.d.). Kindr Attitude Representation. 2018.
    */
    Matrix3f dcm;
    dcm(0,0) = pow(q(3),2) + pow(q(0),2) - pow(q(1),2) - pow(q(2),2);
    dcm(0,1) = 2*(q(0)*q(1) + q(3)*q(2));
    dcm(0,2) = 2*(q(0)*q(2) - q(3)*q(1));
    dcm(1,0) = 2*(q(0)*q(1) - q(3)*q(2));
    dcm(1,1) = pow(q(3),2) - pow(q(0),2) + pow(q(1),2) - pow(q(2),2);
    dcm(1,2) = 2*(q(1)*q(2) + q(3)*q(0));
    dcm(2,0) = 2*(q(0)*q(2) + q(3)*q(1));
    dcm(2,1) = 2*(q(1)*q(2) - q(3)*q(0));
    dcm(2,2) = pow(q(3),2) - pow(q(0),2) - pow(q(1),2) + pow(q(2),2);
    return dcm.transpose();
  }

  casadi::DM CasadiNMPCNodelet::eigen2dm(MatrixXd mat) {
    /*
    Convert from eigen to CasADi's DM input
    */
    casadi::DM dm_mat{std::vector<double>(mat.data(), mat.size() + mat.data())};
    dm_mat = reshape(dm_mat, mat.rows(), mat.cols());

    return dm_mat;
  }

  /* ************************************************************************** */
  void CasadiNMPCNodelet::print_x_des_traj_(int idx) {
    /*
    Validate x_des_traj_
    */
    // std::string my_str;
    // for (int idx0 = 0; idx0 < MATLAB_MAX_ROWS_; idx0++) {
    //   for (int idx1 = 0; idx1 < MPC_NUM_COLS_; idx1++) {
    //     if (idx0 == idx-1) {
    //       my_str.append(std::to_string(x_des_traj_[idx1*MATLAB_MAX_ROWS_ + idx0]));
    //       my_str.append(" ");
    //     }
    //   }
    // }
    // NODELET_INFO_STREAM(my_str);

    IOFormat HeavyFmt(FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");
    std::string sep = "\n----------------------------------------\n";
    NODELET_INFO_STREAM(sep << eigen_x_des_traj_.block(idx,0,1,7).format(HeavyFmt) << sep);
  }

  /* ************************************************************************** */
  void CasadiNMPCNodelet::make_gains(gains gains_YAML){
    /* Make MPC gain options.
    */
    gains gains_permissive_u = {
      100, 5, 0.1, 1000, 50,
      100, 5, 0.1, 1000, 50,
      5, 2, 0.5
    };

    gains gains_strict_u = {
      100, 5, 10, 100, 10,
      100, 5, 10, 100, 10,
      5, 2, 10
    };

    gains gains_cautious_tube_mpc = {
      50, 2, 0.5, 100, 10,
      50, 2, 0.5, 100, 10,
      10, 5, 4
    };

    gains_list_ground_.insert(gains_list_ground_.end(), { gains_YAML, gains_permissive_u, gains_strict_u, gains_cautious_tube_mpc });
    gains_list_iss_.insert(gains_list_iss_.end(), { gains_YAML, gains_permissive_u, gains_strict_u, gains_cautious_tube_mpc });
  }

  /* ************************************************************************** */
  void CasadiNMPCNodelet::switch_gains(int gain_mode){
    /* Switch MPC gains using gains struct.
    */
    gains gains;
    if (ground_.compare("true") == 0) {
      gains = gains_list_ground_[gain_mode];
    }
    else {
      gains = gains_list_iss_[gain_mode];
    }

    Q1 = gains.Q_pos_factor;
    Q2 = gains.Q_pos_factor;
    Q3 = gains.Q_pos_factor;
    Q4 = gains.Q_vel_factor;
    Q5 = gains.Q_vel_factor;
    Q6 = gains.Q_vel_factor;
    R1 = gains.R_factor;
    R2 = gains.R_factor;
    R3 = gains.R_factor;
    QN1 = gains.QN_pos_factor;
    QN2 = gains.QN_pos_factor;
    QN3 = gains.QN_pos_factor;
    QN4 = gains.QN_vel_factor;
    QN5 = gains.QN_vel_factor;
    QN6 = gains.QN_vel_factor;

    Q1_T_ = gains.Q_pos_tube_factor;
    Q2_T_ = gains.Q_pos_tube_factor;
    Q3_T_ = gains.Q_pos_tube_factor;
    Q4_T_ = gains.Q_vel_tube_factor;
    Q5_T_ = gains.Q_vel_tube_factor;
    Q6_T_ = gains.Q_vel_tube_factor;
    R1_T_ = gains.R_tube_factor;
    R2_T_ = gains.R_tube_factor;
    R3_T_ = gains.R_tube_factor;
    QN1_T_ = gains.QN_pos_tube_factor;
    QN2_T_ = gains.QN_pos_tube_factor;
    QN3_T_ = gains.QN_pos_tube_factor;
    QN4_T_ = gains.QN_vel_tube_factor;
    QN5_T_ = gains.QN_vel_tube_factor;
    QN6_T_ = gains.QN_vel_tube_factor;

    Q_pos_anc_factor_ = gains.Q_pos_anc_factor;
    Q_vel_anc_factor_ = gains.Q_vel_anc_factor;
    R_anc_factor_ = gains.R_anc_factor;

    dm_Q1_ = casadi::DM{Q1};
    dm_Q2_ = casadi::DM{Q2};
    dm_Q3_ = casadi::DM{Q3};
    dm_Q4_ = casadi::DM{Q4};
    dm_Q5_ = casadi::DM{Q5};
    dm_Q6_ = casadi::DM{Q6};
    dm_R1_ = casadi::DM{R1};
    dm_R2_ = casadi::DM{R2};
    dm_R3_ = casadi::DM{R3};
    dm_QN1_ = casadi::DM{QN1};
    dm_QN2_ = casadi::DM{QN2};
    dm_QN3_ = casadi::DM{QN3};
    dm_QN4_ = casadi::DM{QN4};
    dm_QN5_ = casadi::DM{QN5};
    dm_QN6_ = casadi::DM{QN6};

    dm_Q1_T_= casadi::DM{Q1_T_};
    dm_Q2_T_= casadi::DM{Q2_T_};
    dm_Q3_T_= casadi::DM{Q3_T_};
    dm_Q4_T_= casadi::DM{Q4_T_};
    dm_Q5_T_= casadi::DM{Q5_T_};
    dm_Q6_T_= casadi::DM{Q6_T_};
    dm_R1_T_= casadi::DM{R1_T_};
    dm_R2_T_= casadi::DM{R2_T_};
    dm_R3_T_= casadi::DM{R3_T_};
    dm_QN1_T_ = casadi::DM{QN1_T_};
    dm_QN2_T_ = casadi::DM{QN2_T_};
    dm_QN3_T_ = casadi::DM{QN3_T_};
    dm_QN4_T_ = casadi::DM{QN4_T_};
    dm_QN5_T_ = casadi::DM{QN5_T_};
    dm_QN6_T_ = casadi::DM{QN6_T_}; // use casadi uniform constructor
  }

  /* ************************************************************************** */
  void CasadiNMPCNodelet::set_mRPI_fallback_values() {
    /* Uses fallback gain, input, and mRPI values for predetermined gain values.
    Only used if mRPI call fails entirely (e.g., bad uc_bound).
    These values are determined using:
    u = [0.15 0.15 0.15]
    */
    using_fallback_mrpi_ = true;

    K_dr_.resize(3, 6);
    K_dr_ <<
    -1.1594,         0,         0,   -4.9716,         0,         0,
          0,   -1.1594,         0,         0,   -4.9716,         0,
          0,         0,   -1.1594,         0,         0,   -4.9716;

    Au_.resize(6, 3);
    Au_ <<
    1,     0,     0,
    0,     1,     0,
    0,     0,     1,
   -1,     0,     0,
    0,    -1,     0,
    0,     0,    -1;

    bu_.resize(6, 1);
    bu_ << 0.0704,    0.0704,    0.0704,    0.0704,    0.0704,    0.0704;  // tightened!

    AZ_.resize(100, 6);
    AZ_ <<
    1.0000,         0,         0,         0,         0,         0,
   -1.0000,         0,         0,         0,         0,         0,
   -0.1289,         0,         0,   -0.9917,         0,         0,
    0.5424,         0,         0,   -0.8401,         0,         0,
         0,         0,         0,   -1.0000,         0,         0,
   -0.5424,         0,         0,    0.8401,         0,         0,
    0.1289,         0,         0,    0.9917,         0,         0,
         0,         0,         0,    1.0000,         0,         0,
         0,    1.0000,         0,         0,         0,         0,
         0,   -1.0000,         0,         0,         0,         0,
         0,   -0.1289,         0,         0,   -0.9917,         0,
         0,    0.5424,         0,         0,   -0.8401,         0,
         0,         0,         0,         0,   -1.0000,         0,
         0,   -0.5424,         0,         0,    0.8401,         0,
         0,    0.1289,         0,         0,    0.9917,         0,
         0,         0,         0,         0,    1.0000,         0,
         0,         0,    1.0000,         0,         0,         0,
         0,         0,   -1.0000,         0,         0,         0,
         0,         0,   -0.1289,         0,         0,   -0.9917,
         0,         0,    0.5424,         0,         0,   -0.8401,
         0,         0,         0,         0,         0,   -1.0000,
         0,         0,   -0.5424,         0,         0,    0.8401,
         0,         0,    0.1289,         0,         0,    0.9917,
         0,         0,         0,         0,         0,    1.0000,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0,
         0,         0,         0,         0,         0,         0;

    bZ_.resize(100, 1);
    bZ_ <<     0.0185, 0.0185, 0.0140, 0.0144, 0.0132, 0.0144, 0.0140, 0.0132, 0.0185, 0.0185, 0.0140, 0.0144, 0.0132, 0.0144, 0.0140, 0.0132, 0.0185, 0.0185, 0.0140, 0.0144, 0.0132, 0.0144, 0.0140, 0.0132,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0,      0;
    dm_K_dr_ = eigen2dm(K_dr_);
    dm_Au_ = eigen2dm(Au_);
    dm_bu_ = eigen2dm(bu_);
    dm_AZ_ = eigen2dm(AZ_);
    dm_bZ_ = eigen2dm(bZ_);
  }
} // end namespace casadi_nmpc