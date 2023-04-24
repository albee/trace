#ifndef CASADI_NMPC_H_
#define CASADI_NMPC_H_

#include <Eigen/Dense>
#include <vector>
#include <fstream>
#include <string.h>
#include <iostream>
#include <boost/numeric/odeint/stepper/runge_kutta_dopri5.hpp>
#include <boost/numeric/odeint/algebra/vector_space_algebra.hpp>
#include <boost/numeric/odeint.hpp>

// ROS
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <ros/package.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include <geometry_msgs/Inertia.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>

// FSW
#include <ff_msgs/ControlState.h>
#include <ff_msgs/FlightMode.h>
#include <ff_msgs/EkfState.h>
#include <ff_msgs/GraphState.h>
#include <ff_msgs/FamCommand.h>
#include <trace_astrobee_interface/ff_nodelet.h>
#include <trace_astrobee_interface/ff_names.h>
#include <trace_astrobee_interface/ff_flight.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <msg_conversions/msg_conversions.h>
#include <pluginlib/class_list_macros.h>

// TRACE
#include <trace_msgs/TDMRPI_msg.h>
#include <trace_msgs/TDMRPI_srv.h>
#include <trace_msgs/TDStatus.h>
#include <trace_msgs/TDCasadiDebug.h>
#include <trace_msgs/TDCasadiStatus.h>
#include <trace_msgs/TDTrajBody.h>
#include "data/eigen_kdl.h"
#include "data/eigen_msg.h"
#include "data/traj_utils.h"
#include "data/csv_read.h"
#include "backup_controller/backup_controller.h"

// CasADi
#include <casadi/casadi.hpp>
#include <cstddef>  // Includes of codegen shared library
#include <cstdlib>

using namespace std;
using namespace Eigen;
using namespace boost::numeric::odeint;

typedef Matrix<float, 1, 7> state_type;  // type for regulation state vector

namespace casadi_nmpc {
struct gains {
  // Standard MPC
  double Q_pos_factor;
  double Q_vel_factor;
  double R_factor;
  double QN_pos_factor;
  double QN_vel_factor;

  // Tube MPC
  double Q_pos_tube_factor;
  double Q_vel_tube_factor;
  double R_tube_factor;
  double QN_pos_tube_factor;
  double QN_vel_tube_factor;

  // ancillary controller
  double Q_pos_anc_factor;
  double Q_vel_anc_factor;
  double R_anc_factor;
};

static std::string CASADI_MPC_LIB = "libcasadi_mpc.so";
static std::string CASADI_ROBUST_TUBE_MPC_LIB = "libcasadi_robust_tube_mpc.so";

static std::string UC_BOUND_TOPIC = "/td/uc_bound/uc_bound";
static std::string TOPIC_TD_TUBE_MPC_TRAJ = "td/tube_mpc/traj";
static std::string TOPIC_TD_TUBE_MPC_TRAJ_BODY = "/td/tube_mpc/traj_body";
static std::string TOPIC_TD_TUBE_MPC_DEBUG = "td/tube_mpc/debug";
static std::string TOPIC_TD_STATUS = "td/status";
static std::string TOPIC_TD_TUBE_MPC_MRPI = "/td/tube_mpc/mrpi";

class CasadiNMPCNodelet : public trace_astrobee_interface::FreeFlyerNodelet {
 public:
  CasadiNMPCNodelet() : trace_astrobee_interface::FreeFlyerNodelet(true) {}  // initialization of FreeFlyerNodelet
  ~CasadiNMPCNodelet() {}

  void Run();  // starts the main ROS loop
  
 // parameters and status info
  std::string control_mode_ = "inactive";  // {track, track_tube, regulate, debug, inactive, unit_test, unit_test_pd}
  std::string state_mode_ = "ekf"; // {ekf, slam}
  std::string ground_ = "false";
  std::string sim_ = "false";
  bool online_update_mode_ = false;
  int gain_mode_ = 0;  // {0, 1, 2, 3, ...}. 0 is always the YAML values.

  bool chaser_coord_ok_ = true;
  bool mrpi_finished_ = false;
  bool traj_finished_ = false;
  bool unit_test_complete_ = false;
  bool using_fallback_mrpi_ = false;
  bool casadi_on_target_ = false;

//   bool use_ekf_dlr_ = false;
//   bool got_ekf_dlr_ = false;
//   double dlr_timeout_period_ = 10.0;
  double tic_startup_;

  // ROS pubs and subs
  ros::Publisher pub_ctl_;
  ros::Publisher pub_debug_;
  ros::Publisher pub_casadi_status_;
  ros::Publisher pub_mrpi_;
  ros::Publisher pub_eigen_x_des_traj_;

  ros::Subscriber sub_ekf_;
//   ros::Subscriber sub_ekf_dlr_;
  ros::Subscriber sub_slam_pos_;
  ros::Subscriber sub_slam_vel_;
  ros::Subscriber sub_slam_targ_att_;
  ros::Subscriber sub_slam_targ_omega_;
  ros::Subscriber sub_x_des_traj_;
  ros::Subscriber sub_x_des_traj_body_;
  ros::Subscriber sub_uc_bound_;
  ros::Subscriber sub_status_;
  ros::Subscriber sub_inertia_;
  std::shared_ptr<std::thread> thread_;  // for subscriber management

  // rates and timing: must match CasADi export!
  double T; // Time horizon, [s]
  int N; // control intervals per time horizon (|x| is 1 greater)

  double traj_rate_;      // rate at which the trajectory is fed, [Hz]
  double MPC_rate_;
  double MPC_dt_;  // MPC timestep [s]
  double control_dt_;    // rate at which the controller expects to be called---cannot be changed, [s]
  double command_rate_ = 62.5;  // rate at which Astrobee firmware expects control inputs, [Hz]
  double casadi_comp_time_;  // rate of latest casadi computation
  double total_comp_time_;

  // system definition
  double mass_; // mass, [kg]

  // inputs
  pd::BackupController pd_control_;  // PD controller class
  Matrix<double, 3, 1> u_mag_; // [N]
  Matrix<double, 3, 1> torque_mag_;  // [N-m]
  double torque_x_ = 0.0;  // torque from ctl [N-m]
  double torque_y_ = 0.0;
  double torque_z_ = 0.0;
  Matrix<double, 6, 1> u_opt_;  // optimal control input [Fx, Fy, Fz, Tx, Ty, Tz]

  // trajectory book-keeping
  int HAVE_TRAJ = 0;
  int WAS_TRACK = 0;
  int N_traj_ = 0; // length of trajectory
  double dt_traj_ = 0.0;  // dt of trajecory
  int traj_idx_ = 0;  // current index of the trajectory to track
  ros::Time t_start_ ;
  double t_elapsed_;
  double t_prev_;  // rostime of previous
  Eigen::Vector3d r_RI_;  // TVR (reference) frame wrt inertial

  // estimates and x_des
  Matrix<double, 6, 1> x_real_;  // current translational state position [x1, x2, x3, x1d, x2d, x3d].T
  Matrix<double, 16, 1> x_real_complete_;  // entire x vector supplied by estimator [x y z  qx qy qz qw  vx vy vz  wx wy wz  ax ay az].T
  MatrixXd x_des_traj_N_;  // length of des traj
  // x_des := [[t x y z xd yd zd qx qy qz qw wx wy wz xdd ydd zdd wxd wyd wzd] ... ]
  MatrixXd eigen_x_des_traj_;        // trajectory in inertial frame. Updated with target attitude estimates.
  MatrixXd eigen_x_des_traj_body_;   // trajectory in target body frame. DOES NOT CHANGE!
  MatrixXd eigen_q_targ_0_hist_;  // history of the nominal target attitude
  MatrixXd eigen_x_des_traj_init_;  // initial inertial frame trajectory

  // regulation
  Vector3f x0_;  // regulation translational state
  Vector4f a0_;  // regulation attitude state
  Matrix<double, 50, 14> eigen_x_des_traj_reg_; // regulation desired trajectory

  // mRPI
  Matrix<double, 6, 1> w_bound_;  // uncertainty bound
  MatrixXd K_dr_ = MatrixXd::Zero(3, 6);  // default initialization
  MatrixXd Au_;
  MatrixXd bu_;
  MatrixXd AZ_;
  MatrixXd bZ_;

  // Target info
  Vector4f q_targ_; // current estimated target attitude
  Vector3f omega_targ_; // current estimate target omega
  Eigen::Vector3d targ_offset_;  // Target position offsets
  std::vector<float> J_vec_;  // [3,3] inertia tensor

  // Tube MPC and MPC vars
  // QN indicate terminal weight. Q is state; R is input.
  std::vector<gains> gains_list_ground_;
  std::vector<gains> gains_list_iss_;

  // individual input weights
  double Q1;  // standard MPC
  double Q2;
  double Q3;
  double Q4;
  double Q5;
  double Q6;
  double R1;
  double R2;
  double R3;
  double QN1;
  double QN2;
  double QN3;
  double QN4;
  double QN5;
  double QN6;
  double Q1_T_;  // tube MPC
  double Q2_T_;
  double Q3_T_;
  double Q4_T_;
  double Q5_T_;
  double Q6_T_;
  double R1_T_;
  double R2_T_;
  double R3_T_;
  double QN1_T_;
  double QN2_T_;
  double QN3_T_;
  double QN4_T_;
  double QN5_T_;
  double QN6_T_;

  double Q_pos_anc_factor_;  // ancillary controller
  double Q_vel_anc_factor_;
  double R_anc_factor_;

  // CasADi variables
  casadi::DM dm_u_mag_;
  casadi::DM dm_m_;
  casadi::DM dm_Au_;
  casadi::DM dm_bu_;
  casadi::DM dm_AZ_;
  casadi::DM dm_bZ_;
  casadi::DM dm_K_dr_;
  casadi::DM dm_Q1_;
  casadi::DM dm_Q2_;
  casadi::DM dm_Q3_;
  casadi::DM dm_Q4_;
  casadi::DM dm_Q5_;
  casadi::DM dm_Q6_;
  casadi::DM dm_R1_;
  casadi::DM dm_R2_;
  casadi::DM dm_R3_;
  casadi::DM dm_QN1_;
  casadi::DM dm_QN2_;
  casadi::DM dm_QN3_;
  casadi::DM dm_QN4_;
  casadi::DM dm_QN5_;
  casadi::DM dm_QN6_;
  casadi::DM dm_Q1_T_;
  casadi::DM dm_Q2_T_;
  casadi::DM dm_Q3_T_;
  casadi::DM dm_Q4_T_;
  casadi::DM dm_Q5_T_;
  casadi::DM dm_Q6_T_;
  casadi::DM dm_R1_T_;
  casadi::DM dm_R2_T_;
  casadi::DM dm_R3_T_;
  casadi::DM dm_QN1_T_;
  casadi::DM dm_QN2_T_;
  casadi::DM dm_QN3_T_;
  casadi::DM dm_QN4_T_;
  casadi::DM dm_QN5_T_;
  casadi::DM dm_QN6_T_;

  // CasADi functions
  casadi::Function mpc_func_casadi_;  // C versions
  casadi::Function tube_mpc_func_casadi_;
  casadi::Function tube_mpc_func_serialized_;  // serialized graph versions
  casadi::Function mpc_func_serialized_;

  // ROAM
  runge_kutta_dopri5<state_type, double, state_type, double, vector_space_algebra> stepper_;  // dynamic propagator stuff
  Eigen::Matrix3f J_targ_;

  // function defs
  void Initialize(ros::NodeHandle* nh);
  void MPC_timer_callback(const ros::TimerEvent&);
  void command_timer_callback(const ros::TimerEvent&);
  void update_u_opt_(std::string control_mode_);
  void select_closest_setpoint();
  Vector3d calc_torques_PD(bool regulate);
  Vector3d call_pd_and_print();

  // tests
  void run_debug();
  void unit_test_mpc();
  void run_unit_test();

  // mpc calls
  std::tuple<Vector3d, Vector3d, Vector3d, Matrix<double, 6, 1>> call_tube_mpc_func_casadi();
  Vector3d call_mpc_func_casadi(bool regulate);
  Eigen::MatrixXd get_setpoints();
  void prepare_mrpi(Eigen::MatrixXd w, Eigen::MatrixXd u_max, double dt, double mass, double Q_pos_anc, double Q_vel_anc, double R_anc);

  // ROS pubs and subs
  void status_callback(const trace_msgs::TDStatus::ConstPtr& msg);
  void ekf_callback(const ff_msgs::EkfState::ConstPtr msg);
//   void ekf_dlr_callback(const ekf_dlr::EKF_Obs_State::ConstPtr msg);
  void w_bound_callback(const std_msgs::Float64MultiArray::ConstPtr uc_bound);
  void x_des_traj_callback(const std_msgs::Float64MultiArray::ConstPtr msg);
  void publish_casadi_status();
  void publish_mrpi();
  void publish_eigen_x_des_traj();
  void publish_debug(Matrix<double, 6, 1> u_t_idx, Vector3d u0_mpc, Vector3d u0_dr, Matrix<double, 6, 1> x_nom);
  void publish_ctl(tf2::Vector3 F_xyz_B, tf2::Vector3 T_xyz_B);

  // utils
  std::tuple <tf2::Vector3, tf2::Vector3> check_input_limits(tf2::Vector3 F_xyz_B, tf2::Vector3 T_xyz_B);
  void make_gains(gains gains_YAML);
  void switch_gains(int gain_mode);
  void set_mRPI_fallback_values();
  std::string quat2str(tf2::Quaternion q);
  void get_regulation_and_uc_bound_parameters();
  void get_all_YAML_parameters();
  void update_regulation_setpoint(Vector3f x0, Vector4f a0);
  Eigen::Matrix3f q2dcm(const Vector4f &q);
  void read_traj_from_file(std::string traj_filename);
  casadi::DM eigen2dm(MatrixXd mat);
  void print_x_des_traj_(int idx);

  // ROAM functions
  void slam_pose_callback(const geometry_msgs::PoseWithCovariance::ConstPtr msg);
  void slam_twist_callback(const geometry_msgs::TwistWithCovariance::ConstPtr msg);
  void slam_targ_att_callback(const geometry_msgs::PoseWithCovariance::ConstPtr msg);
  void slam_targ_omega_callback(const geometry_msgs::TwistWithCovariance::ConstPtr msg);
  void x_des_traj_body_callback(const trace_msgs::TDTrajBody::ConstPtr msg);
  void inertia_callback(const geometry_msgs::Inertia::ConstPtr inertia_msg);
  void update_inertial_traj();
  void read_traj_standard_format(std::string traj_filename);

  class dynamic_step_ {
      Matrix3f J_param;
      public:
          dynamic_step_( Matrix3f G) : J_param( G ) {}

          void operator()(state_type const&x, state_type&dx, const float &t) const {
              VectorXf q = x.head(4);
              Vector3f w = x.segment(4,3);
              Vector3f M = VectorXf::Zero(3);   // Assuming no external torques on target
              Matrix3f J_inv;

              J_inv = J_param.inverse();

              // Quaternion kinematics
              MatrixXf B(4,4);
              B(0,0) = 0.0;
              B(0,1) = w(2);
              B(0,2) = -w(1);
              B(0,3) = w(0);

              B(1,0) = -w(2);
              B(1,1) = 0.0;
              B(1,2) = w(0);
              B(1,3) = w(1);

              B(2,0) = w(1);
              B(2,1) = -w(0);
              B(2,2) = 0.0;
              B(2,3) = w(2);

              B(3,0) = -w(0);
              B(3,1) = -w(1);
              B(3,2) = -w(2);
              B(3,3) = 0.0;

              VectorXf qdot = 0.5 * B * q;

              // Euler's equations for rigid bodies
              Vector3f wdot = J_inv * (M - w.cross(J_param * w));

              dx(0) = qdot(0);
              dx(1) = qdot(1);
              dx(2) = qdot(2);
              dx(3) = qdot(3);
              dx(4) = wdot(0);
              dx(5) = wdot(1);
              dx(6) = wdot(2);
          }
  };

  class dynamic_step_ground_ {
      Matrix3f J_param;
      public:
          dynamic_step_ground_( Matrix3f G) : J_param( G ) {}

          void operator()(state_type const&x, state_type&dx, const float &t) const {
              VectorXf q = x.head(4);
              Vector3f w = x.segment(4,3);

              // Quaternion kinematics
              MatrixXf B(4,4);
              B(0,0) = 0.0;
              B(0,1) = w(2);
              B(0,2) = -w(1);
              B(0,3) = w(0);

              B(1,0) = -w(2);
              B(1,1) = 0.0;
              B(1,2) = w(0);
              B(1,3) = w(1);

              B(2,0) = w(1);
              B(2,1) = -w(0);
              B(2,2) = 0.0;
              B(2,3) = w(2);

              B(3,0) = -w(0);
              B(3,1) = -w(1);
              B(3,2) = -w(2);
              B(3,3) = 0.0;

              VectorXf qdot = 0.5 * B * q;

              // Euler's equations for rigid bodies
              Vector3f wdot;
              wdot << 0.0, 0.0, 0.0;

              dx(0) = qdot(0);
              dx(1) = qdot(1);
              dx(2) = qdot(2);
              dx(3) = qdot(3);
              dx(4) = wdot(0);
              dx(5) = wdot(1);
              dx(6) = wdot(2);
          }
  };
};
}  // end namespace casadi_nmpc
#endif  // CASADI_NMPC_H_
