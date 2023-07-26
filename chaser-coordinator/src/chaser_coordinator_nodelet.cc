/**
 * @file chaser_coordinator_nodelet.cc
 * @author Keenan Albee and Charles Oestreich
 * @brief Coordination node for Chaser satellite operation. Includes events that cannot be handled by chaser_asap.py
 * @version 0.1
 * @date 2023-04-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <Eigen/Dense>
#include <vector>
#include <fstream>
#include <chrono>
#include <boost/numeric/odeint/stepper/runge_kutta_dopri5.hpp>
#include <boost/numeric/odeint/algebra/vector_space_algebra.hpp>
#include <boost/numeric/odeint/integrate/integrate.hpp>
#include <string.h>

// ROS
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/package.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/Inertia.h>
#include <std_srvs/SetBool.h>

// FSW
#include <ff_util/ff_nodelet.h>
#include <ff_util/ff_names.h>
#include <ff_util/ff_flight.h>

#include <ff_msgs/ControlState.h>
#include <ff_msgs/FlightMode.h>
#include <ff_msgs/FamCommand.h>
#include <ff_msgs/SignalState.h>
#include <msg_conversions/msg_conversions.h>

// TRACE
#include <trace_msgs/TDStatus.h>
#include <trace_msgs/TDTestNumber.h>
#include <trace_msgs/TDMotionPlannerInterfaceStatus.h>
#include <trace_msgs/TDUCBoundStatus.h>
#include <trace_msgs/TDCasadiStatus.h>
#include <trace_msgs/TDSlamInfo.h>
#include <trace_msgs/TDTrajBody.h>

#include "data/eigen_kdl.h"
#include "data/eigen_msg.h"
#include "data/traj_utils.h"
#include "data/csv_read.h"

float EPSILON = std::numeric_limits<float>::epsilon();  // for floating point comparison

using namespace Eigen;
using namespace boost::numeric::odeint;

typedef Matrix<float, 1, 7> state_type;

namespace chaser_coordinator {

std::string TOPIC_TD_TUBE_MPC_TRAJ = "td/tube_mpc/traj";
std::string TOPIC_TD_TUBE_MPC_TRAJ_BODY = "/td/tube_mpc/traj_body";
std::string TOPIC_TD_TARG_ATT0 = "/td/motion_planner_interface/target_traj_pose0";
std::string TOPIC_TD_TARG_OMEGA0 = "/td/motion_planner_interface/target_traj_twist0";
std::string TOPIC_TD_GNC_CTL_COMMAND = "/td/gnc/ctl/command";
std::string UC_BOUND_TOPIC = "/td/uc_bound/uc_bound";
std::string TOPIC_TD_STATUS = "td/status";
std::string TOPIC_TD_TEST_NUMBER = "td/test_number";

class ChaserCoordinatorNodelet : public ff_util::FreeFlyerNodelet {
 public:
  ChaserCoordinatorNodelet() : ff_util::FreeFlyerNodelet(true) {}  // don't do anything ROS-related in the constructor!
  ~ChaserCoordinatorNodelet() {}

 private:
  /*
  eigen_x_des_traj_ =
  [t x y z xd yd zd qw qx qy qz wx wy wz xdd ydd zdd wxd wyd wzd
  ...
  ]
  */
  std::shared_ptr<std::thread> thread_;
  ros::Publisher pub_traj_default_;
  ros::Publisher pub_x_des_traj_;
  ros::Publisher pub_x_des_traj_body_;
  ros::Publisher pub_flight_mode_;
  ros::Publisher pub_uc_bound_;
  ros::Publisher pub_status_;
  ros::Publisher pub_signal_;
  // debug only
  ros::Publisher target_est_pose_pub_;
  ros::Publisher target_est_twist_pub_;

  ros::Subscriber sub_slam_targ_att_;
  ros::Subscriber sub_slam_targ_omega_;
  ros::Subscriber sub_flight_mode_;
  ros::Subscriber sub_td_gnc_;
  ros::Subscriber sub_targ_att0_;
  ros::Subscriber sub_targ_omega0_;
  ros::Subscriber sub_ekf_;
  ros::Subscriber sub_test_number_;
  ros::Subscriber sub_motion_planner_interface_status_;
  ros::Subscriber sub_uc_bound_status_;
  ros::Subscriber sub_casadi_status_;
  ros::Subscriber sub_slam_info_;
  ros::Subscriber sub_inertia_;

  // ff_util::FreeFlyerActionClient<ff_msgs::ControlAction> client_control_;

  ros::Timer status_timer_;
  ros::Timer gnc_ctl_setpoint_timer_;

  float reg_time_;

  ros::ServiceClient serv_ctl_enable_;

  ff_msgs::FlightMode flight_mode_;
  MatrixXd eigen_x_des_traj_;
  MatrixXd eigen_x_des_traj_body_;
  int Nf_ = 1;  // length of eigen_x_des_traj_;
  int ACTIVATE_DEFAULT_REGULATION_ = 0;  // whether regulation is allowed

  // Target info
  Eigen::Vector4f q_targ0_;  // sent to mp
  Eigen::Vector3f omega_targ0_;  // sent to mp
  Vector4f q_targ_; // current estimated target attitude
  Vector3f omega_targ_; // current estimate target omega

  // Target position offsets
  Eigen::Vector3d targ_offset_;
  Eigen::Vector3d r_RI_;  // TVR (reference) frame wrt inertial

  // Ekf state
  Eigen::Matrix<double, 16, 1> x_real_complete_;  // entire x vector supplied by estimator

  // Parameters
  std::string controller_ = "default";  // controller to send commands to
  std::string ground_ = "false";  // whether or not this is a ground test
  std::string sim_ = "false";
  std::string traj_filename_ = "";
  std::string flight_mode_check_;
  bool use_tube_mpc_;  // {true or false}
  bool use_pd_;  // {true or false}, overrides mpc TODO
  Eigen::Vector3d x0_;
  Eigen::Vector4d a0_;

  bool uc_bound_unit_test_complete_ = false;
  bool casadi_unit_test_complete_ = false;

  // Status parameters
  int test_number_ = 0;
  bool chaser_coord_ok_ = true;
  bool slam_activate_ = false;
  bool motion_planner_interface_activate_ = false;
  bool uc_bound_activate_ = false;
  bool chaser_regulate_finished_ = false;
  bool slam_converged_ = false;
  bool inertia_estimated_ = false;
  bool motion_plan_finished_ = false;
  float motion_plan_wait_time_ = 0.0;
  bool uc_bound_finished_ = false;
  bool mrpi_finished_ = false;
  bool traj_finished_ = false;
  bool default_control_ = true;
  std::string td_control_mode_ = "inactive";
  std::string td_state_mode_ = "ekf";
  std::string td_flight_mode_ = "nominal";
  int td_gain_mode_ = 0;
  bool test_finished_ = false;
  bool test_slam_spoof_ = false;
  bool online_update_mode_ = false;

  // Test-specific ISS parameters
  std::string test_control_mode_ = "inactive";
  std::string test_state_mode_ = "ekf";
  std::string test_LUT_ = "y";
  std::string test_tumble_type_ = "triaxial";
  int LUT_param_ = 4;  // this causes the param to be triaxial spin case for unit tests

  // Rates
  double instruct_check_rate_;  // [Hz]
  double gnc_ctl_setpoint_rate_;  // [Hz]

  // Regulation thresholds
  float pos_reg_thresh_;
  float vel_reg_thresh_;
  float att_reg_thresh_;
  float omega_reg_thresh_;

  // pause length after regulation and before test.
  float after_reg_pause_;

  // dynamic propagator stuff
  runge_kutta_dopri5<state_type, double, state_type, double, vector_space_algebra> stepper_;
  Eigen::Matrix3f J_;

  // signal lights!
  typedef struct signal_sent {
    int slam_activate = 0;
    int motion_planner_interface_activate = 0;
    int control_mode_activate = 0;
    int test_finished = 0;
  } signal_sent;

  ff_msgs::SignalState signal_light_state;
  signal_sent signal_sent_;

  ///
  /// Intializers and ROS callbacks
  ///
  /* ************************************************************************** */
  void Initialize(ros::NodeHandle* nh) {
    /* This is called when the nodelet is loaded into the nodelet manager
    */

    // If in the sim, we need the robot namespace for default topics
    ros::param::getCached("/td/sim", sim_);
    ros::param::getCached("/td/ground", ground_);
    x_real_complete_ << 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0; // dummy data until estimator publishes

    // rates, pauses, instructions
    ros::param::getCached("/td/chaser_coordinator/instruct_check_rate", instruct_check_rate_);
    ros::param::getCached("/td/chaser_coordinator/gnc_ctl_setpoint_rate_", gnc_ctl_setpoint_rate_);
    ros::param::getCached("/td/chaser/after_reg_pause", after_reg_pause_);
    ros::param::getCached("/td/chaser_coordinator/use_tube_mpc", use_tube_mpc_);
    ros::param::getCached("/td/chaser_coordinator/use_pd_", use_pd_);

    // chaser regulation
    ros::param::getCached("/td/chaser/x_start", x0_(0));
    ros::param::getCached("/td/chaser/y_start", x0_(1));
    ros::param::getCached("/td/chaser/z_start", x0_(2));
    ros::param::getCached("/td/chaser/qx_start", a0_(0));
    ros::param::getCached("/td/chaser/qy_start", a0_(1));
    ros::param::getCached("/td/chaser/qz_start", a0_(2));
    ros::param::getCached("/td/chaser/qw_start", a0_(3));

    // target pos IC (INERTIAL frame)
    ros::param::getCached("/td/chaser/targ_offset_x", targ_offset_(0));
    ros::param::getCached("/td/chaser/targ_offset_y", targ_offset_(1));
    ros::param::getCached("/td/chaser/targ_offset_z", targ_offset_(2));

    // translation of TVR frame w/r to ISS frame
    ros::param::getCached("/td/r_RI_ISS_x", r_RI_(0));
    ros::param::getCached("/td/r_RI_ISS_y", r_RI_(1));
    ros::param::getCached("/td/r_RI_ISS_z", r_RI_(2));

    ros::param::getCached("/td/chaser_coordinator/pos_reg_thresh", pos_reg_thresh_);
    ros::param::getCached("/td/chaser_coordinator/vel_reg_thresh", vel_reg_thresh_);
    ros::param::getCached("/td/chaser_coordinator/att_reg_thresh", att_reg_thresh_);
    ros::param::getCached("/td/chaser_coordinator/omega_reg_thresh", omega_reg_thresh_);

    ros::param::getCached("/td/chaser_coordinator/reg_time", reg_time_);
    // NODELET_INFO_STREAM(instruct_check_rate_ << " " << gnc_ctl_setpoint_rate_);

    std::vector<float> J_vec;
    ros::param::getCached("/td/uc_bound/target_J", J_vec);
    int k = 0;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
          J_(i,j) = J_vec[k];
          k++;
      }
    }

    // publishers
    pub_flight_mode_ = nh->advertise<ff_msgs::FlightMode>(TOPIC_MOBILITY_FLIGHT_MODE, 1, true);  // FlightMode
    pub_traj_default_ = nh->advertise<ff_msgs::ControlState>(TOPIC_GNC_CTL_SETPOINT, 5, false);  // setpoints for default Astrobee controller
    pub_x_des_traj_ = nh->advertise<std_msgs::Float64MultiArray>(TOPIC_TD_TUBE_MPC_TRAJ, 5, true);  // full traj---used once for TubeMPC
    pub_x_des_traj_body_ = nh->advertise<trace_msgs::TDTrajBody>(TOPIC_TD_TUBE_MPC_TRAJ_BODY, 5, true);  // full traj in body frame --- used once for TubeMPC
    pub_uc_bound_ = nh->advertise<std_msgs::Float64MultiArray>(UC_BOUND_TOPIC, 5, true);
    pub_status_ = nh->advertise<trace_msgs::TDStatus>(TOPIC_TD_STATUS, 5, true);
    pub_signal_ = nh->advertise<ff_msgs::SignalState>("signals", 1, true);  // TOPIC_SIGNALS is a latched topic

    // for debugging only
    target_est_pose_pub_ = nh->advertise<geometry_msgs::PoseWithCovariance>("/td/mit_slam/target_pose", 1);
    target_est_twist_pub_ = nh->advertise<geometry_msgs::TwistWithCovariance>("/td/mit_slam/target_twist", 1);

    // subscribers
    sub_flight_mode_= nh->subscribe<ff_msgs::FlightMode>(TOPIC_MOBILITY_FLIGHT_MODE, 5,
      boost::bind(&ChaserCoordinatorNodelet::flight_mode_callback, this, _1));  // flight mode setter
    sub_ekf_ = nh->subscribe<ff_msgs::EkfState>("/gnc/ekf", 5,
      boost::bind(&ChaserCoordinatorNodelet::ekf_callback, this, _1));
    sub_targ_att0_ = nh->subscribe<geometry_msgs::PoseWithCovariance>(TOPIC_TD_TARG_ATT0, 5,
      boost::bind(&ChaserCoordinatorNodelet::targ_att0_callback, this, _1));
    sub_targ_omega0_ = nh->subscribe<geometry_msgs::TwistWithCovariance>(TOPIC_TD_TARG_OMEGA0, 5,
      boost::bind(&ChaserCoordinatorNodelet::targ_omega0_callback, this, _1));
    sub_test_number_ = nh->subscribe<trace_msgs::TDTestNumber>(TOPIC_TD_TEST_NUMBER, 5,
      boost::bind(&ChaserCoordinatorNodelet::test_num_callback, this, _1));
    sub_motion_planner_interface_status_ = nh->subscribe<trace_msgs::TDMotionPlannerInterfaceStatus>("/td/motion_planner_interface/status", 5,
      boost::bind(&ChaserCoordinatorNodelet::motion_planner_interface_status_callback, this, _1));
    sub_uc_bound_status_ = nh->subscribe<trace_msgs::TDUCBoundStatus>("/td/uc_bound/status", 5,
      boost::bind(&ChaserCoordinatorNodelet::uc_bound_status_callback, this, _1));
    sub_casadi_status_ = nh->subscribe<trace_msgs::TDCasadiStatus>("td/casadi_nmpc/status", 5,
      boost::bind(&ChaserCoordinatorNodelet::casadi_status_callback, this, _1));
    sub_slam_info_ = nh->subscribe<trace_msgs::TDSlamInfo>("/td/mit_slam/timing_info", 5,
      boost::bind(&ChaserCoordinatorNodelet::slam_info_callback, this, _1));
    sub_inertia_ = nh->subscribe<geometry_msgs::Inertia>("/td/mit_slam/inertia", 5,
      boost::bind(&ChaserCoordinatorNodelet::inertia_callback, this, _1));
    sub_slam_targ_att_ = nh->subscribe<geometry_msgs::PoseWithCovariance>("/td/mit_slam/target_pose", 5,
      boost::bind(&ChaserCoordinatorNodelet::slam_targ_att_callback, this, _1));
    sub_slam_targ_omega_ = nh->subscribe<geometry_msgs::TwistWithCovariance>("/td/mit_slam/target_twist", 5,
      boost::bind(&ChaserCoordinatorNodelet::slam_targ_omega_callback, this, _1));

    // actions
    // See line 542 in choreographer_nodelet.cc for example setup
    //client_control_.SetFeedbackCallback(std::bind(&ChaserCoordinatorNodelet::ControlFeedbackCallback, this, std::placeholders::_1));
    //client_control_.SetResultCallback(std::bind(&ChaserCoordinatorNodelet::ControlResultCallback, this, std::placeholders::_1, std::placeholders::_2)));
    //client_control_.SetConnectedCallback(std::bind(&ChaserCoordinatorNodelet::ControlConnectedCallback, this));
    // client_control_.Create(nh, ACTION_GNC_CTL_CONTROL);

    // services
    serv_ctl_enable_ = nh->serviceClient<std_srvs::SetBool>(SERVICE_GNC_CTL_ENABLE);

    NODELET_INFO_STREAM("[CHASER COORD] Initialized");

    thread_.reset(new std::thread(&chaser_coordinator::ChaserCoordinatorNodelet::Run, this));
  }

  /* ************************************************************************** */
  void Run() {
    /*
    Interpret the `/test_number` topic and run tests. High-level test management
    takes place here, and is delegated out to other nodes.

    Each test function is intended to be run just ONCE per test number received!
    */

    //ros::Duration(5.0).sleep();  // sleep for a bit to allow all nodelets to launch
    ros::NodeHandle MTNH = getMTNodeHandle();  // multithread within nodelet

    ros::Rate sleep_rate(10.0);

    ros::Timer gnc_ctl_setpoint_rate = MTNH.createTimer(ros::Duration(1/gnc_ctl_setpoint_rate_),
      boost::bind(&ChaserCoordinatorNodelet::gnc_ctl_setpoint_callback, this, _1));  // send commands

    ros::Timer status_timer_rate = MTNH.createTimer(ros::Duration(0.2),
      boost::bind(&ChaserCoordinatorNodelet::PublishTDStatus, this, _1));  // send commands (5 Hz)

    while (ros::ok() && test_number_ == 0) {
      ros::spinOnce();
      sleep_rate.sleep();
    }

    if (test_number_ != 1) {
      if (!ff_util::FlightUtil::GetFlightMode(flight_mode_, td_flight_mode_)) {
          return;
      }  // create a nominal FlightMode
      pub_flight_mode_.publish(flight_mode_);// Publish default flight mode so CTL/FAM will actually perform actuation

    }
    else {
      if (!ff_util::FlightUtil::GetFlightMode(flight_mode_, "off")) {
          return;
      }  // create a nominal FlightMode
      pub_flight_mode_.publish(flight_mode_);// Publish default flight mode so CTL/FAM will actually perform actuation
    }

    ros::Duration(2.0).sleep();  // Pause so flight mode actually gets registered

    while (ros::ok()) {
      if (!test_finished_) {
        // Standard tests/full pipeline
        if (test_number_ > 100) {
          process_test_number();  // set parameters
          td_control_mode_  = "regulate";
          td_state_mode_ = test_state_mode_;
          ros::Duration(0.4).sleep();
          disable_default_ctl();
          check_regulate();
          ros::Duration(after_reg_pause_).sleep();  // pause between regulation and beginning of test
          ros::param::getCached("/td/chaser_traj_file", traj_filename_);

          if (traj_filename_.empty()) {
            chaser_coord_ok_ = false;
          }
          else {
           testISSLUT();
          }
        }

        else if (test_number_ == 3) {
          td_gain_mode_ = 0;
          td_control_mode_ = "regulate";
          td_state_mode_ = "ekf";
          ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate/ekf settings before disabling default controller.
          disable_default_ctl();
          check_regulate();
          ros::Duration(after_reg_pause_).sleep();  // pause between regulation and beginning of test
          ros::param::getCached("/td/chaser_traj_file", traj_filename_);
          if (traj_filename_.empty()) {
            chaser_coord_ok_ = false;
          }
          else {
           test3();
          }
        }

        else if (test_number_ == 4) {
          td_gain_mode_ = 0;
          td_control_mode_ = "regulate";
          td_state_mode_ = "ekf";
          ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate/ekf settings before disabling default controller.
          disable_default_ctl();
          check_regulate();
         ros::Duration(after_reg_pause_).sleep();  // pause between regulation and beginning of test
         ros::param::getCached("/td/chaser_traj_file", traj_filename_);
         if (traj_filename_.empty()) {
           chaser_coord_ok_ = false;
         }
         else {
           test4();
         }
        }

        else if (test_number_ == 5) {
         td_gain_mode_ = 0;
         td_control_mode_ = "regulate";
         td_state_mode_ = "ekf";
         ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate/ekf settings before disabling default controller.
         disable_default_ctl();
         check_regulate();
         ros::Duration(after_reg_pause_).sleep();  // pause between regulation and beginning of test
         ros::param::getCached("/td/chaser_traj_file", traj_filename_);
         if (traj_filename_.empty()) {
           chaser_coord_ok_ = false;
         }
         else {
           test5();
         }
        }

        else if (test_number_ == 6) {
          ACTIVATE_DEFAULT_REGULATION_ = 1;
          check_regulate();
          ACTIVATE_DEFAULT_REGULATION_ = 0;
          td_gain_mode_ = 0;
          ros::Duration(after_reg_pause_).sleep();  // pause between regulation and beginning of test
          ros::param::getCached("/td/chaser_traj_file", traj_filename_);
          if (traj_filename_.empty()) {
            chaser_coord_ok_ = false;
         }
         else {
           test6();
         }
        }

        else if (test_number_ == 7) {
          //ACTIVATE_DEFAULT_REGULATION_ = 1;
          //check_regulate();
          //ACTIVATE_DEFAULT_REGULATION_ = 0;
          td_gain_mode_ = 0;
          td_control_mode_ = "regulate";
          td_state_mode_ = "ekf";
          ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate/ekf settings before disabling default controller.
          disable_default_ctl();
          check_regulate();
         ros::Duration(after_reg_pause_).sleep();  // pause between regulation and beginning of test
         ros::param::getCached("/td/chaser_traj_file", traj_filename_);
         if (traj_filename_.empty()) {
           chaser_coord_ok_ = false;
         }
         else {
           test7();
         }
        }

        else if (test_number_ == 8) {
         td_gain_mode_ = 0;
         td_control_mode_ = "regulate";
         td_state_mode_ = "ekf";
         ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate/ekf settings before disabling default controller.
         disable_default_ctl();
         check_regulate();
         ros::Duration(after_reg_pause_).sleep();  // pause between regulation and beginning of test
         ros::param::getCached("/td/chaser_traj_file", traj_filename_);
         if (traj_filename_.empty()) {
           chaser_coord_ok_ = false;
         }
         else {
           test8();
         }
        }

        else if (test_number_ == 9) {
         td_gain_mode_ = 0;
         td_control_mode_ = "regulate";
         td_state_mode_ = "ekf";
         ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate/ekf settings before disabling default controller.
         disable_default_ctl();
         check_regulate();
         ros::Duration(after_reg_pause_).sleep();  // pause between regulation and beginning of test
         ros::param::getCached("/td/chaser_traj_file", traj_filename_);
         if (traj_filename_.empty()) {
           chaser_coord_ok_ = false;
         }
         else {
           test9();
         }
        }

        else if (test_number_ == 10) {
         td_gain_mode_ = 0;
         td_control_mode_ = "regulate";
         td_state_mode_ = "ekf";
         ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate/ekf settings before disabling default controller.
         disable_default_ctl();
         check_regulate();
         ros::Duration(after_reg_pause_).sleep();  // pause between regulation and beginning of test
         ros::param::getCached("/td/chaser_traj_file", traj_filename_);
         if (traj_filename_.empty()) {
           chaser_coord_ok_ = false;
         }
         else {
           test10();
         }
        }

        else if (test_number_ == 11) {
         td_gain_mode_ = 0;
         td_control_mode_ = "regulate";
         td_state_mode_ = "ekf";
         online_update_mode_ = true;
         ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate/ekf settings before disabling default controller.
         disable_default_ctl();
         check_regulate();
         ros::Duration(after_reg_pause_).sleep();  // pause between regulation and beginning of test
         ros::param::getCached("/td/chaser_traj_file", traj_filename_);
         if (traj_filename_.empty()) {
           chaser_coord_ok_ = false;
         }
         else {
           test11();
         }
        }

        else if (test_number_ == 12) {
          td_gain_mode_ = 0;
          td_control_mode_ = "regulate";
          td_state_mode_ = "ekf";
          test_slam_spoof_ = true;
          ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate/ekf settings before disabling default controller.
          disable_default_ctl();
          check_regulate();
          ros::Duration(after_reg_pause_).sleep();  // pause between regulation and beginning of test
          ros::param::getCached("/td/chaser_traj_file", traj_filename_);
          if (traj_filename_.empty()) {
            chaser_coord_ok_ = false;
          }
          else {
           test12();
          }
        }
        else if (test_number_ == 13) {  // slam spoof, ekf state mode, no online updates
          td_gain_mode_ = 0;
          td_control_mode_ = "regulate";
          td_state_mode_ = "ekf";
          ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate/ekf settings before disabling default controller.
          disable_default_ctl();
          check_regulate();
          ros::Duration(after_reg_pause_).sleep();  // pause between regulation and beginning of test
          ros::param::getCached("/td/chaser_traj_file", traj_filename_);
          if (traj_filename_.empty()) {
            chaser_coord_ok_ = false;
          }
          else {
           test_slam_spoof_ = true;
           test8();
          }
        }

       else if (test_number_ == 14) {  // slam spoof, slam state mode, online updates
         td_gain_mode_ = 0;
         td_control_mode_ = "regulate";
         td_state_mode_ = "ekf";
         online_update_mode_ = true;
         ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate/ekf settings before disabling default controller.
         disable_default_ctl();
         check_regulate();
         ros::Duration(after_reg_pause_).sleep();  // pause between regulation and beginning of test
         ros::param::getCached("/td/chaser_traj_file", traj_filename_);
         if (traj_filename_.empty()) {
           chaser_coord_ok_ = false;
         }
         else {
          test_slam_spoof_ = true;
          test11();
         }
       }
        else if (test_number_ == 15) {
         td_gain_mode_ = 0;
         td_control_mode_ = "regulate";
         td_state_mode_ = "ekf";
         ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate/ekf settings before disabling default controller.
         disable_default_ctl();
         check_regulate();
         ros::Duration(after_reg_pause_).sleep();  // pause between regulation and beginning of test
         ros::param::getCached("/td/chaser_traj_file", traj_filename_);
         if (traj_filename_.empty()) {
           chaser_coord_ok_ = false;
         }
         else {
           test15();
         }
        }
        else if (test_number_ == 16) {
         td_gain_mode_ = 0;
         td_control_mode_ = "regulate";
         td_state_mode_ = "ekf";
         ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate/ekf settings before disabling default controller.
         disable_default_ctl();
         check_regulate();
         ros::Duration(after_reg_pause_).sleep();  // pause between regulation and beginning of test
         ros::param::getCached("/td/chaser_traj_file", traj_filename_);
         if (traj_filename_.empty()) {
           chaser_coord_ok_ = false;
         }
         else {
           test16();
         }
        }
        /****** DEBUG ******/
        else if (test_number_ == 77) {  // debug traj_body test
         td_gain_mode_ = 0;
         td_control_mode_ = "regulate";
         td_state_mode_ = "ekf";
         ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate/ekf settings before disabling default controller.
         disable_default_ctl();
         // check_regulate();
         // ros::Duration(after_reg_pause_).sleep();  // pause between regulation and beginning of test
         ros::param::getCached("/td/chaser_traj_file", traj_filename_);
         if (traj_filename_.empty()) {
           chaser_coord_ok_ = false;
         }
         else {
           debug_traj_body();
         }
        }
        /****** DEBUG ******/

       // reset parameter values
       online_update_mode_ = false;
       td_gain_mode_ = 0;
       td_state_mode_ = "ekf";
      }

      ros::spinOnce();
      sleep_rate.sleep();
    }
  }

  /* ************************************************************************** */
  void process_test_number(){
    // Process test number logic if greater than 100 (i.e., 8-digit tests)
    if (test_number_ > 100) {
      std::string test_number_str = std::to_string(test_number_);

      // Axis and tumble selection xx(xxxxxx)
      if (test_number_str[0] == '1' && test_number_str[1] == '1') {
        test_LUT_ = "x";
        test_tumble_type_ = "triaxial";
        LUT_param_ = 1;
      }
      else if (test_number_str[0] == '1' && test_number_str[1] == '2') {
        test_LUT_ = "x";
        test_tumble_type_ = "flat_spin";
        LUT_param_ = 2;
      }
      else if (test_number_str[0] == '1' && test_number_str[1] == '3') {
        test_LUT_ = "x";
        test_tumble_type_ = "stopped";
        LUT_param_ = 3;
      }
      else if (test_number_str[0] == '2' && test_number_str[1] == '1') {
        test_LUT_ = "y";
        test_tumble_type_ = "triaxial";
        LUT_param_ = 4;
      }
      else if (test_number_str[0] == '2' && test_number_str[1] == '2') {
        test_LUT_ = "y";
        test_tumble_type_ = "flat_spin";
        LUT_param_ = 5;
      }
      else if (test_number_str[0] == '2' && test_number_str[1] == '3') {
        test_LUT_ = "y";
        test_tumble_type_ = "stopped";
        LUT_param_ = 6;
      }
      else if (test_number_str[0] == '3' && test_number_str[1] == '1') {
        test_LUT_ = "z";
        test_tumble_type_ = "triaxial";
        LUT_param_ = 7;
      }
      else if (test_number_str[0] == '3' && test_number_str[1] == '2') {
        test_LUT_ = "z";
        test_tumble_type_ = "flat_spin";
        LUT_param_ = 8;
      }
      else if (test_number_str[0] == '3' && test_number_str[1] == '3') {
        test_LUT_ = "z";
        test_tumble_type_ = "stopped";
        LUT_param_ = 9;
      }

      // Parameter settings xx(xxxxxx)
      // controller
      if (test_number_str[2] == '1') {  // standard MPC
        test_control_mode_ = "track";
      }
      else if (test_number_str[2] == '2') {  // tube MPC
        test_control_mode_ = "track_tube";
      }

      // gains
      if (test_number_str[3] == '1') {
        td_gain_mode_ = 0;
      }
      else if (test_number_str[3] == '2') {
        td_gain_mode_ = 1;
      }
      else if (test_number_str[3] == '3') {
        td_gain_mode_ = 2;
      }
      else if (test_number_str[3] == '4') {
        td_gain_mode_ = 3;
      }

      // planner
      if (test_number_str[4] == '1') {  // default
        LUT_param_ += 9;
      }
      else if (test_number_str[4] == '2') {  // shorter wait time
        LUT_param_ = LUT_param_;
      }

      // state mode --> replaced with toggle for PD on Target (no effect for Chaser)
      test_state_mode_ = "ekf";  // always use gnc/ekf localization

      // spoofing
      if (test_number_str[6] == '1') {  // no spoof
        test_slam_spoof_ = false;
      }
      else if (test_number_str[6] == '2') {  // spoof
        test_slam_spoof_ = true;
      }

      // online updating
      if (test_number_str[7] == '1') {  // no online update
        online_update_mode_ = false;
      }
      else if (test_number_str[7] == '2') {  // online update
        online_update_mode_ = true;
      }
    }
  }

  ///***************************
  /// Tests
  ///***************************
  int testISSLUT() {
    /* ISS standard test/full pipeline with different LUTs/tumbles/settings
    */
    NODELET_INFO_STREAM("[CHASER COORD]: Performing full ISS test.");

    ros::Rate sleep_rate(10.0);

  //  if (sim_.compare("true") == 0) {
  //    ros::Duration(10.0).sleep();  // Allow target to settle into tumble.
  //  }

    // Beginning estimation loop for inertia
    if (!test_slam_spoof_) {
      NODELET_INFO_STREAM("[CHASER COORD]: Beginning inertia estimation.");
      slam_activate_ = true;
      float est_count = 0;
      //while (ros::ok() && est_count < estimator_inertia_length) {
      //  est_count = est_count + sleep_count;
      //  sleep_rate.sleep();
      //}
      while (ros::ok() && !inertia_estimated_) {
        ros::spinOnce();
        sleep_rate.sleep();
      }
      NODELET_INFO_STREAM("[CHASER COORD]: Inertia estimation finished.");
    }
    else {
      slam_activate_ = true;
      // Wait a bit so SLAM for sure is producing spoofed estimates before motion planner is called
      auto slam_spoof_start = std::chrono::high_resolution_clock::now();
      auto slam_spoof_now = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> slam_spoof_elapsed = slam_spoof_now - slam_spoof_start;
      while (ros::ok() && std::chrono::duration<double>(slam_spoof_elapsed).count() < 15.0) {
        sleep_rate.sleep();
        slam_spoof_now = std::chrono::high_resolution_clock::now();
        slam_spoof_elapsed = slam_spoof_now - slam_spoof_start;
      }
      inertia_estimated_ = true;
    }

    // Call motion planner based on pre-defined inputs
    NODELET_INFO_STREAM("[CHASER COORD]: Beginning motion planning.");
    motion_planner_interface_activate_ = true;
    // Wait for motion_planner_interface to finish motion plan
    while (ros::ok() && !motion_plan_finished_) {
      ros::spinOnce();
      sleep_rate.sleep();
    }

    // Read the timing file to get the post-motion plan wait time, or -1 for fault
    std::string time_file = traj_filename_ + "result_time_0.dat";
    double time_value = 0;
    std::ifstream indata;
    indata.open(time_file);
    while(indata.good()) {
      indata >> time_value;
    }
    indata.close();

    motion_plan_wait_time_ = time_value;

    if (time_value != -1) {
      // Read in and publish the trajectory produced by the motion planner
      read_traj_standard_format("caroline", 0);
      NODELET_INFO_STREAM("[CHASER COORD]: Motion plan published.");

      // Begin timing for post motion planner time
      auto post_motion_plan_start = std::chrono::high_resolution_clock::now();

      // Call UC bound based on motion plan
      NODELET_INFO_STREAM("[CHASER COORD]: Beginning UC bound calculation.");
      uc_bound_activate_ = true;
      // Wait for uc bound to finish
      while(ros::ok() && !uc_bound_finished_) {
        ros::spinOnce();
        sleep_rate.sleep();
      }
      NODELET_INFO_STREAM("[CHASER COORD]: UC bound calculation finished.");

      // Calculate mRPI for tube MPC
      // Calculate mRPI for tube MPC
      NODELET_INFO_STREAM("[CHASER COORD]: Beginning mRPI calculation.");
      while(ros::ok() && !mrpi_finished_) {
        ros::spinOnce();
        sleep_rate.sleep();
      }
      NODELET_INFO_STREAM("[CHASER COORD]: mRPI calculation finished.");

      // Wait until post motion planner time is complete
      auto post_motion_plan_now = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> post_motion_plan_elapsed = post_motion_plan_now - post_motion_plan_start;
      while (ros::ok() && std::chrono::duration<double>(post_motion_plan_elapsed).count() < time_value) {
        sleep_rate.sleep();
        post_motion_plan_now = std::chrono::high_resolution_clock::now();
        post_motion_plan_elapsed = post_motion_plan_now - post_motion_plan_start;
      }

      NODELET_INFO_STREAM("[CHASER COORD]: Sending body trajectory...");
      send_body_traj();
      ros::Duration(0.2).sleep(); // wait a moment to ensure received by casadi_nmpc

      // Follow trajectory using controller and EKF
      NODELET_INFO_STREAM("[CHASER COORD]: Beginning to follow trajectory using controller.");
      td_control_mode_ = test_control_mode_;
      td_state_mode_ = test_state_mode_;

      // Wait for trajectory to finish
      while(ros::ok() && !traj_finished_) {
        ros::spinOnce();
        sleep_rate.sleep();
      }
      NODELET_INFO_STREAM("[CHASER COORD]: Trajectory finished, test complete.");
      td_control_mode_ = "regulate";
    }
    else {
      NODELET_INFO_STREAM("[CHASER COORD]: Motion planner fault.");
    }
    test_finished_ = true;

    return 1;
  }

  // Tube MPC unit test
  int test3() {
    NODELET_INFO_STREAM("[CHASER COORD]: Performing tube MPC unit test.");

    // sample uc_bound
    MatrixXf eig_uc_bound_mat(4, 3);
    eig_uc_bound_mat << 0.0331, 0.0260, 0.0537,
                        0.0331, -0.0260, -0.0537,
                        0.0069, 0.0055, 0.0073,
                        -0.0069, -0.0055, -0.0073;  // reasonable values (pretty large, causes fallback)

    // mRPI trigger
    std_msgs::Float64MultiArray uc_bound_mat;
    tf::matrixEigenToMsg(eig_uc_bound_mat, uc_bound_mat);

    ros::Rate loop_rate(10.0);

    // BEGIN TEST
    // Read in the preset trajectory
    read_traj_standard_format("caroline", 0);
    send_body_traj();

    pub_uc_bound_.publish(uc_bound_mat);

    // Calculate mRPI for tube MPC
    NODELET_INFO_STREAM("[CHASER COORD]: Beginning mRPI calculation.");
    while(ros::ok() && !mrpi_finished_) {
      ros::spinOnce();
      loop_rate.sleep();
    }
    NODELET_INFO_STREAM("[CHASER COORD]: mRPI calculation finished.");
    td_gain_mode_ = 1;
    (use_tube_mpc_ == true) ? td_control_mode_ = "track_tube" : td_control_mode_ = "track";
    td_state_mode_ = "ekf";

    // Track the ref traj using tube MPC
    NODELET_INFO_STREAM("[CHASER COORD]: Beginning tube MPC unit test.");
    while (ros::ok() && !traj_finished_) {
      ros::spinOnce();
      loop_rate.sleep();
    }

    test_finished_ = true;
    td_control_mode_ = "regulate";
    td_gain_mode_ = 0;
    return 1;
  }

  // PD attitude control test
  int test4() {
    NODELET_INFO_STREAM("[CHASER COORD]: Performing PD controller unit test.");

    // Read in the trajectory produced by the motion planner
    read_traj_standard_format("caroline", 0);
    send_body_traj();

    (use_tube_mpc_ == true) ? td_control_mode_ = "track_tube" : td_control_mode_ = "track";
    td_state_mode_ = "ekf";

    ros::Rate loop_rate(10.0);

    while (ros::ok() && !traj_finished_) {
      ros::spinOnce();
      loop_rate.sleep();
    }

    NODELET_INFO_STREAM("[CHASER COORD]: PD controller unit test complete.");

    test_finished_ = true;
    td_control_mode_ = "regulate";
    return 1;
  }

  // Motion planner unit test (no reliance on SLAM)
  int test5() {
    NODELET_INFO_STREAM("[CHASER COORD]: Performing motion planner unit test.");

    ros::Rate sleep_rate(10.0);

    // Call motion planner based on pre-defined inputs
    NODELET_INFO_STREAM("[CHASER COORD]: Beginning motion planning.");
    motion_planner_interface_activate_ = true;
    // Wait for motion_planner_interface to finish motion plan
    while (ros::ok() && !motion_plan_finished_) {
      ros::spinOnce();
      sleep_rate.sleep();
    }

    // Read the timing file to get the post-motion plan wait time, or -1 for fault
    std::string time_file = traj_filename_ + "result_time_0.dat";
    double time_value = 0;
    std::ifstream indata;
    indata.open(time_file);
    while(indata.good()) {
      indata >> time_value;
    }
    indata.close();

    motion_plan_wait_time_ = time_value;

    if (time_value != -1) {
      // Read in and publish the trajectory produced by the motion planner
      read_traj_standard_format("caroline", 0);
      NODELET_INFO_STREAM("[CHASER COORD]: Motion plan published.");

      // Begin timing for post motion planner time
      auto post_motion_plan_start = std::chrono::high_resolution_clock::now();

      // Call UC bound based on motion plan
      NODELET_INFO_STREAM("[CHASER COORD]: Beginning UC bound calculation.");
      // sample uc_bound
      //MatrixXf eig_uc_bound_mat(4, 3);
      //eig_uc_bound_mat << 0.0331, 0.0260, 0.0537,
      //                    0.0331, -0.0260, -0.0537,
      //                    0.0069, 0.0055, 0.0073,
      //                    -0.0069, -0.0055, -0.0073;

      // mRPI trigger
      //std_msgs::Float64MultiArray uc_bound_mat;
      //tf::matrixEigenToMsg(eig_uc_bound_mat, uc_bound_mat);
      //uc_bound_finished_ = true;
      //pub_uc_bound_.publish(uc_bound_mat);
      // Wait for uc bound to finish
      uc_bound_activate_ = true;
      while(ros::ok() && !uc_bound_finished_) {
        ros::spinOnce();
        sleep_rate.sleep();
      }
      NODELET_INFO_STREAM("[CHASER COORD]: UC bound calculation finished.");

      // Calculate mRPI for tube MPC
      NODELET_INFO_STREAM("[CHASER COORD]: Beginning mRPI calculation.");
      while(ros::ok() && !mrpi_finished_) {
        ros::spinOnce();
        sleep_rate.sleep();
      }
      NODELET_INFO_STREAM("[CHASER COORD]: mRPI calculation finished.");

      // Wait until post motion planner time is complete
      auto post_motion_plan_now = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> post_motion_plan_elapsed = post_motion_plan_now - post_motion_plan_start;
      while (ros::ok() && std::chrono::duration<double>(post_motion_plan_elapsed).count() < time_value) {
        sleep_rate.sleep();
        post_motion_plan_now = std::chrono::high_resolution_clock::now();
        post_motion_plan_elapsed = post_motion_plan_now - post_motion_plan_start;
      }

      NODELET_INFO_STREAM("[CHASER COORD]: Sending body trajectory...");
      send_body_traj();
      ros::Duration(0.2).sleep(); // wait a moment to ensure received by casadi_nmpc

      // Follow trajectory using controller and EKF
      NODELET_INFO_STREAM("[CHASER COORD]: Beginning to follow trajectory using controller.");
      (use_tube_mpc_ == false) ? td_control_mode_ = "track_tube" : td_control_mode_ = "track";
      td_state_mode_ = "ekf";
      // Wait for trajectory to finish
      while(ros::ok() && !traj_finished_) {
        ros::spinOnce();
        sleep_rate.sleep();
      }
      NODELET_INFO_STREAM("[CHASER COORD]: Trajectory finished, test complete. Regulating.");
      td_control_mode_ = "regulate";
    }
    else {
      NODELET_INFO_STREAM("[CHASER COORD]: Motion planner fault.");
    }
    test_finished_ = true;

    return 1;
  }

  // Inertia estimation test
  int test6() {
    NODELET_INFO_STREAM("[CHASER COORD]: Performing inertia estimation unit test.");

    if (sim_.compare("true") == 0) {
      ros::Duration(15.0).sleep();  // Allow target to settle into tumble.
    }

    ros::Rate sleep_rate(10.0);

    slam_activate_ = true;

    auto inertia_test_start = std::chrono::high_resolution_clock::now();

    // Wait until inertia test time is complete
    auto inertia_test_now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> inertia_test_elapsed = inertia_test_now - inertia_test_start;
    float test_time = 120.0;
    if (ground_.compare("true") == 0) {
      test_time = 120.0;
    }
    else {
      test_time = 360.0;
    }
    while (ros::ok() && std::chrono::duration<double>(inertia_test_elapsed).count() < test_time) {
      ros::spinOnce();
      sleep_rate.sleep();
      inertia_test_now = std::chrono::high_resolution_clock::now();
      inertia_test_elapsed = inertia_test_now - inertia_test_start;
    }

    NODELET_INFO_STREAM("[CHASER COORD]: Inertia estimation unit test complete.");

    slam_activate_ = false;

    test_finished_ = true;
    return 1;
  }

  // SLAM unit test
  int test7() {
    NODELET_INFO_STREAM("[CHASER COORD]: Performing SLAM unit test.");

  //  if (sim_.compare("true") == 0) {
  //    ros::Duration(10.0).sleep();  // Allow target to settle into tumble.
  //  }

    slam_activate_ = true;

    ros::Rate sleep_rate(10.0);

    MatrixXd input_data = csv::load_csv<MatrixXd>(traj_filename_);  // read in CSV trajectory

    ros::Duration(15.0).sleep();  // pause for initial chaser estimation stuff

    // Publish inspection trajectory for casadi to follow
    eigen_x_des_traj_.resize(input_data.cols(), 14);
    for (int i = 0; i < input_data.cols(); i++) {
      eigen_x_des_traj_(i, 0) = input_data(0,i);
      eigen_x_des_traj_(i, 1) = input_data(1,i);
      eigen_x_des_traj_(i, 2) = input_data(2,i);
      eigen_x_des_traj_(i, 3) = input_data(3,i);
      eigen_x_des_traj_(i, 4) = input_data(4,i);
      eigen_x_des_traj_(i, 5) = input_data(5,i);
      eigen_x_des_traj_(i, 6) = input_data(6,i);

      eigen_x_des_traj_(i, 7) = input_data(7, i);
      eigen_x_des_traj_(i, 8) = input_data(8, i);
      eigen_x_des_traj_(i, 9) = input_data(9, i);
      eigen_x_des_traj_(i, 10) = input_data(10, i);

      eigen_x_des_traj_(i, 11) = 0.0;
      eigen_x_des_traj_(i, 12) = 0.0;
      eigen_x_des_traj_(i, 13) = 0.0;
    }

    std_msgs::Float64MultiArray eigen_x_des_traj_msg;
    tf::matrixEigenToMsg(eigen_x_des_traj_, eigen_x_des_traj_msg);

    pub_x_des_traj_.publish(eigen_x_des_traj_msg);

    td_control_mode_ = "track";
    td_state_mode_ = "ekf";

    auto inertia_test_start = std::chrono::high_resolution_clock::now();

    // Wait until inertia test time is complete
    auto inertia_test_now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> inertia_test_elapsed = inertia_test_now - inertia_test_start;

    while (ros::ok() && std::chrono::duration<double>(inertia_test_elapsed).count() < 300.0) {
      ros::spinOnce();
      sleep_rate.sleep();
      inertia_test_now = std::chrono::high_resolution_clock::now();
      inertia_test_elapsed = inertia_test_now - inertia_test_start;
    }

    /*
    float t = 0.0;
    int col_cntr = 0;
    float traj_rate = 5; // Hz
    float dt = 1 / traj_rate;
    ros::Rate loop_rate(traj_rate);

    while (col_cntr < input_data.cols()) {
      geometry_msgs::Point traj_pos;
      geometry_msgs::Vector3 traj_vel;
      geometry_msgs::Quaternion traj_quat;
      geometry_msgs::Vector3 traj_omega;
      geometry_msgs::Vector3 traj_accel;

      traj_pos.x = input_data(0,col_cntr);
      traj_pos.y = input_data(1,col_cntr);
      traj_pos.z = input_data(2,col_cntr);
      traj_vel.x = input_data(3,col_cntr);
      traj_vel.y = input_data(4,col_cntr);
      traj_vel.z = input_data(5,col_cntr);
      traj_quat.w = input_data(9,col_cntr);
      traj_quat.x = input_data(6,col_cntr);
      traj_quat.y = input_data(7,col_cntr);
      traj_quat.z = input_data(8,col_cntr);
      //traj_omega.x = input_data(10,col_cntr);
      //traj_omega.y = input_data(11,col_cntr);
      //traj_omega.z = input_data(12,col_cntr);
      traj_omega.x = 0.0;
      traj_omega.y = 0.0;
      traj_omega.z = 0.0;

      geometry_msgs::Vector3 traj_alpha;
      traj_alpha.x = 0.0;
      traj_alpha.y = 0.0;
      traj_alpha.z = 0.0;

          // Publish the tumbling traj topic
      static ff_msgs::ControlState inspect;
      inspect.when = ros::Time::now();;
      inspect.pose.position = traj_pos;
      inspect.pose.orientation = traj_quat;
      inspect.twist.linear = traj_vel;
      inspect.twist.angular = traj_omega;
      inspect.accel.linear = traj_accel;
      inspect.accel.angular = traj_alpha;

      NODELET_DEBUG_STREAM("Pub traj pos y: " << input_data(1, col_cntr));

      pub_traj_default_.publish(inspect);

      t = t + dt;
      col_cntr++;
      loop_rate.sleep();
    }

    */

    NODELET_INFO_STREAM("[CHASER COORD]: SLAM unit test complete.");

    slam_activate_ = false;
    test_finished_ = true;
    td_control_mode_ = "regulate";
    return 1;
  }

  // Full traj., nominal MPC and EKF state mode
  int test8() {

    NODELET_INFO_STREAM("[CHASER COORD]: Performing full trajectory test.");

    ros::Rate sleep_rate(10.0);

  //  if (sim_.compare("true") == 0) {
  //    ros::Duration(10.0).sleep();  // Allow target to settle into tumble.
  //  }

    // Beginning estimation loop for inertia
    if (!test_slam_spoof_) {
      NODELET_INFO_STREAM("[CHASER COORD]: Beginning inertia estimation.");
      slam_activate_ = true;
      float est_count = 0;
      //while (ros::ok() && est_count < estimator_inertia_length) {
      //  est_count = est_count + sleep_count;
      //  sleep_rate.sleep();
      //}
      while (ros::ok() && !inertia_estimated_) {
        ros::spinOnce();
        sleep_rate.sleep();
      }
      NODELET_INFO_STREAM("[CHASER COORD]: Inertia estimation finished.");
    }
    else {
      slam_activate_ = true;
      // Wait a bit so SLAM for sure is producing spoofed estimates before motion planner is called
      auto slam_spoof_start = std::chrono::high_resolution_clock::now();
      auto slam_spoof_now = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> slam_spoof_elapsed = slam_spoof_now - slam_spoof_start;
      while (ros::ok() && std::chrono::duration<double>(slam_spoof_elapsed).count() < 15.0) {
        sleep_rate.sleep();
        slam_spoof_now = std::chrono::high_resolution_clock::now();
        slam_spoof_elapsed = slam_spoof_now - slam_spoof_start;
      }
      inertia_estimated_ = true;
    }

    // Call motion planner based on pre-defined inputs
    NODELET_INFO_STREAM("[CHASER COORD]: Beginning motion planning.");
    motion_planner_interface_activate_ = true;
    // Wait for motion_planner_interface to finish motion plan
    while (ros::ok() && !motion_plan_finished_) {
      ros::spinOnce();
      sleep_rate.sleep();
    }

    // Read the timing file to get the post-motion plan wait time, or -1 for fault
    std::string time_file = traj_filename_ + "result_time_0.dat";
    double time_value = 0;
    std::ifstream indata;
    indata.open(time_file);
    while(indata.good()) {
      indata >> time_value;
    }
    indata.close();

    motion_plan_wait_time_ = time_value;

    if (time_value != -1) {
      // Read in and publish the trajectory produced by the motion planner
      read_traj_standard_format("caroline", 0);
      NODELET_INFO_STREAM("[CHASER COORD]: Motion plan published.");

      // Begin timing for post motion planner time
      auto post_motion_plan_start = std::chrono::high_resolution_clock::now();

      // Call UC bound based on motion plan
      NODELET_INFO_STREAM("[CHASER COORD]: Beginning UC bound calculation.");
      uc_bound_activate_ = true;
      // Wait for uc bound to finish
      while(ros::ok() && !uc_bound_finished_) {
        ros::spinOnce();
        sleep_rate.sleep();
      }
      NODELET_INFO_STREAM("[CHASER COORD]: UC bound calculation finished.");

      // Calculate mRPI for tube MPC
      // Calculate mRPI for tube MPC
      NODELET_INFO_STREAM("[CHASER COORD]: Beginning mRPI calculation.");
      while(ros::ok() && !mrpi_finished_) {
        ros::spinOnce();
        sleep_rate.sleep();
      }
      NODELET_INFO_STREAM("[CHASER COORD]: mRPI calculation finished.");

      // Wait until post motion planner time is complete
      auto post_motion_plan_now = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> post_motion_plan_elapsed = post_motion_plan_now - post_motion_plan_start;
      while (ros::ok() && std::chrono::duration<double>(post_motion_plan_elapsed).count() < time_value) {
        sleep_rate.sleep();
        post_motion_plan_now = std::chrono::high_resolution_clock::now();
        post_motion_plan_elapsed = post_motion_plan_now - post_motion_plan_start;
      }

      NODELET_INFO_STREAM("[CHASER COORD]: Sending body trajectory...");
      send_body_traj();
      ros::Duration(0.2).sleep(); // wait a moment to ensure received by casadi_nmpc

      // Follow trajectory using controller and EKF
      NODELET_INFO_STREAM("[CHASER COORD]: Beginning to follow trajectory using controller.");
      use_tube_mpc_ = false;
      (use_tube_mpc_ == true) ? td_control_mode_ = "track_tube" : td_control_mode_ = "track";
      td_state_mode_ = "ekf";
      // Wait for trajectory to finish
      while(ros::ok() && !traj_finished_) {
        ros::spinOnce();
        sleep_rate.sleep();
      }
      NODELET_INFO_STREAM("[CHASER COORD]: Trajectory finished, test complete. Regulating.");
      td_control_mode_ = "regulate";
    }
    else {
      NODELET_INFO_STREAM("[CHASER COORD]: Motion planner fault.");
    }
    test_finished_ = true;

    return 1;
  }

  // Full traj., tube MPC and EKF state mode
  int test9() {

    NODELET_INFO_STREAM("[CHASER COORD]: Performing full trajectory test.");

    ros::Rate sleep_rate(10.0);

  //  if (sim_.compare("true") == 0) {
//      ros::Duration(10.0).sleep();  // Allow target to settle into tumble.
  //  }

    // Beginning estimation loop for inertia
    NODELET_INFO_STREAM("[CHASER COORD]: Beginning inertia estimation.");
    slam_activate_ = true;
    float est_count = 0;
    //while (ros::ok() && est_count < estimator_inertia_length) {
    //  est_count = est_count + sleep_count;
    //  sleep_rate.sleep();
    //}
    while (ros::ok() && !inertia_estimated_) {
      ros::spinOnce();
      sleep_rate.sleep();
    }
    NODELET_INFO_STREAM("[CHASER COORD]: Inertia estimation finished.");

    // Call motion planner based on pre-defined inputs
    NODELET_INFO_STREAM("[CHASER COORD]: Beginning motion planning.");
    motion_planner_interface_activate_ = true;
    // Wait for motion_planner_interface to finish motion plan
    while (ros::ok() && !motion_plan_finished_) {
      ros::spinOnce();
      sleep_rate.sleep();
    }

    // Read the timing file to get the post-motion plan wait time, or -1 for fault
    std::string time_file = traj_filename_ + "result_time_0.dat";
    double time_value = 0;
    std::ifstream indata;
    indata.open(time_file);
    while(indata.good()) {
      indata >> time_value;
    }
    indata.close();

    motion_plan_wait_time_ = time_value;

    if (time_value != -1) {
      // Read in and publish the trajectory produced by the motion planner
      read_traj_standard_format("caroline", 0);
      NODELET_INFO_STREAM("[CHASER COORD]: Motion plan published.");

      // Begin timing for post motion planner time
      auto post_motion_plan_start = std::chrono::high_resolution_clock::now();

      // Call UC bound based on motion plan
      NODELET_INFO_STREAM("[CHASER COORD]: Beginning UC bound calculation.");
      uc_bound_activate_ = true;
      // Wait for uc bound to finish
      while(ros::ok() && !uc_bound_finished_) {
        ros::spinOnce();
        sleep_rate.sleep();
      }
      NODELET_INFO_STREAM("[CHASER COORD]: UC bound calculation finished.");

      // Calculate mRPI for tube MPC
      // Calculate mRPI for tube MPC
      NODELET_INFO_STREAM("[CHASER COORD]: Beginning mRPI calculation.");
      while(ros::ok() && !mrpi_finished_) {
        ros::spinOnce();
        sleep_rate.sleep();
      }
      NODELET_INFO_STREAM("[CHASER COORD]: mRPI calculation finished.");

      // Wait until post motion planner time is complete
      auto post_motion_plan_now = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> post_motion_plan_elapsed = post_motion_plan_now - post_motion_plan_start;
      while (ros::ok() && std::chrono::duration<double>(post_motion_plan_elapsed).count() < time_value) {
        sleep_rate.sleep();
        post_motion_plan_now = std::chrono::high_resolution_clock::now();
        post_motion_plan_elapsed = post_motion_plan_now - post_motion_plan_start;
      }

      NODELET_INFO_STREAM("[CHASER COORD]: Sending body trajectory...");
      send_body_traj();
      ros::Duration(0.2).sleep(); // wait a moment to ensure received by casadi_nmpc

      // Follow trajectory using controller and EKF
      NODELET_INFO_STREAM("[CHASER COORD]: Beginning to follow trajectory using controller.");
      use_tube_mpc_ = true;
      (use_tube_mpc_ == true) ? td_control_mode_ = "track_tube" : td_control_mode_ = "track";
      td_state_mode_ = "ekf";
      // Wait for trajectory to finish
      while(ros::ok() && !traj_finished_) {
        ros::spinOnce();
        sleep_rate.sleep();
      }
      NODELET_INFO_STREAM("[CHASER COORD]: Trajectory finished, test complete.");
      td_control_mode_ = "regulate";
    }
    else {
      NODELET_INFO_STREAM("[CHASER COORD]: Motion planner fault.");
    }
    test_finished_ = true;

    return 1;
  }

  // Full traj., tube MPC and EKF state mode, HazCam recording
  int test10() {
    int result = test9();

    return result;
  }

  // Full traj., tube MPC and SLAM state mode
  int test11() {

    NODELET_INFO_STREAM("[CHASER COORD]: Performing full trajectory test.");

    ros::Rate sleep_rate(10.0);

  //  if (sim_.compare("true") == 0) {
//      ros::Duration(10.0).sleep();  // Allow target to settle into tumble.
  //  }

    // Beginning estimation loop for inertia
    if (!test_slam_spoof_) {
      NODELET_INFO_STREAM("[CHASER COORD]: Beginning inertia estimation.");
      slam_activate_ = true;
      float est_count = 0;
      //while (ros::ok() && est_count < estimator_inertia_length) {
      //  est_count = est_count + sleep_count;
      //  sleep_rate.sleep();
      //}
      while (ros::ok() && !inertia_estimated_) {
        ros::spinOnce();
        sleep_rate.sleep();
      }
      NODELET_INFO_STREAM("[CHASER COORD]: Inertia estimation finished.");
    }
    else {
      slam_activate_ = true;
      // Wait a bit so SLAM for sure is producing spoofed estimates before motion planner is called
      auto slam_spoof_start = std::chrono::high_resolution_clock::now();
      auto slam_spoof_now = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> slam_spoof_elapsed = slam_spoof_now - slam_spoof_start;
      while (ros::ok() && std::chrono::duration<double>(slam_spoof_elapsed).count() < 15.0) {
        sleep_rate.sleep();
        slam_spoof_now = std::chrono::high_resolution_clock::now();
        slam_spoof_elapsed = slam_spoof_now - slam_spoof_start;
      }
      inertia_estimated_ = true;
    }

    // Call motion planner based on pre-defined inputs
    NODELET_INFO_STREAM("[CHASER COORD]: Beginning motion planning.");
    motion_planner_interface_activate_ = true;
    // Wait for motion_planner_interface to finish motion plan
    while (ros::ok() && !motion_plan_finished_) {
      ros::spinOnce();
      sleep_rate.sleep();
    }

    // Read the timing file to get the post-motion plan wait time, or -1 for fault
    std::string time_file = traj_filename_ + "result_time_0.dat";
    double time_value = 0;
    std::ifstream indata;
    indata.open(time_file);
    while(indata.good()) {
      indata >> time_value;
    }
    indata.close();

    motion_plan_wait_time_ = time_value;

    if (time_value != -1) {
      // Read in and publish the trajectory produced by the motion planner
      read_traj_standard_format("caroline", 0);
      NODELET_INFO_STREAM("[CHASER COORD]: Motion plan published.");

      // Begin timing for post motion planner time
      auto post_motion_plan_start = std::chrono::high_resolution_clock::now();

      // Call UC bound based on motion plan
      NODELET_INFO_STREAM("[CHASER COORD]: Beginning UC bound calculation.");
      uc_bound_activate_ = true;
      // Wait for uc bound to finish
      while(ros::ok() && !uc_bound_finished_) {
        ros::spinOnce();
        sleep_rate.sleep();
      }
      NODELET_INFO_STREAM("[CHASER COORD]: UC bound calculation finished.");

      // Calculate mRPI for tube MPC
      // Calculate mRPI for tube MPC
      NODELET_INFO_STREAM("[CHASER COORD]: Beginning mRPI calculation.");
      while(ros::ok() && !mrpi_finished_) {
        ros::spinOnce();
        sleep_rate.sleep();
      }
      NODELET_INFO_STREAM("[CHASER COORD]: mRPI calculation finished.");

      // Wait until post motion planner time is complete
      auto post_motion_plan_now = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> post_motion_plan_elapsed = post_motion_plan_now - post_motion_plan_start;
      while (ros::ok() && std::chrono::duration<double>(post_motion_plan_elapsed).count() < time_value) {
        sleep_rate.sleep();
        post_motion_plan_now = std::chrono::high_resolution_clock::now();
        post_motion_plan_elapsed = post_motion_plan_now - post_motion_plan_start;
      }

      NODELET_INFO_STREAM("[CHASER COORD]: Sending body trajectory...");
      send_body_traj();
      ros::Duration(0.2).sleep(); // wait a moment to ensure received by casadi_nmpc

      // Follow trajectory using controller and EKF
      NODELET_INFO_STREAM("[CHASER COORD]: Beginning to follow trajectory using controller.");
      use_tube_mpc_ = true;
      (use_tube_mpc_ == true) ? td_control_mode_ = "track_tube" : td_control_mode_ = "track";
      td_state_mode_ = "slam";
      // Wait for trajectory to finish
      while(ros::ok() && !traj_finished_) {
        ros::spinOnce();
        sleep_rate.sleep();
      }
      NODELET_INFO_STREAM("[CHASER COORD]: Trajectory finished, test complete.");
      td_control_mode_ = "regulate";
    }
    else {
      NODELET_INFO_STREAM("[CHASER COORD]: Motion planner fault.");
    }
    test_finished_ = true;

    return 1;
  }

  int test12() {
    /* Tube MPC unit test with fancy trajectory (21x)
    Equivalent call is:
    ./mpMIT 14 1 13 0 0 0 7828.0 17023.3 397.1 -2171.4 397.1 124825.70 344.2 -2171.4 344.2 129112.2 0 -1.5 0 0.4436 0.5777 -0.4975 -0.5016 -0.1489 3.535 3.535 /path
    */
    NODELET_INFO_STREAM("[CHASER COORD]: Performing tube MPC unit test.");

    // sample uc_bound
    MatrixXf eig_uc_bound_mat(4, 3);
    eig_uc_bound_mat << 0.005, 0.005, 0.005,
                        0.005, 0.005, 0.005,
                        0.0025, 0.0025, 0.0025,
                        -0.0025, -0.0025, 0.0025;  // +pos; -pos; +vel; -vel, per control_dt;

    // mRPI trigger
    std_msgs::Float64MultiArray uc_bound_mat;
    tf::matrixEigenToMsg(eig_uc_bound_mat, uc_bound_mat);

    ros::Rate loop_rate(10.0);

    // BEGIN TEST
    // Read in the trajectory
    read_traj_standard_format("caroline", 0);
    send_body_traj();

    pub_uc_bound_.publish(uc_bound_mat);

    // Calculate mRPI for tube MPC
    NODELET_INFO_STREAM("[CHASER COORD]: Beginning mRPI calculation.");
    while(ros::ok() && !mrpi_finished_) {
      ros::spinOnce();
      loop_rate.sleep();
    }
    NODELET_INFO_STREAM("[CHASER COORD]: mRPI calculation finished.");

    use_tube_mpc_ = true;
    (use_tube_mpc_ == true) ? td_control_mode_ = "track_tube" : td_control_mode_ = "track";
    td_state_mode_ = "ekf";

    // Track the ref traj using tube MPC
    NODELET_INFO_STREAM("[CHASER COORD]: Beginning tube MPC unit test.");
    while (ros::ok() && !traj_finished_) {
      ros::spinOnce();
      loop_rate.sleep();
    }

    test_finished_ = true;
    // td_control_mode_ = "regulate";
    return 1;
  }

  int test15() {
    /* "MIT" test trajectory for standard MPC.
    */

    // sample uc_bound (not used in this test though)
    MatrixXf eig_uc_bound_mat(4, 3);
    eig_uc_bound_mat << 0.005, 0.005, 0.005,
                        0.005, 0.005, 0.005,
                        0.0025, 0.0025, 0.0025,
                        -0.0025, -0.0025, 0.0025;  // +pos; -pos; +vel; -vel, per control_dt

    // mRPI trigger
    std_msgs::Float64MultiArray uc_bound_mat;
    tf::matrixEigenToMsg(eig_uc_bound_mat, uc_bound_mat);

    ros::Rate loop_rate(10.0);

    // BEGIN TEST
    // Read in the trajectory
    read_traj_standard_format("caroline", 0);
    send_body_traj();

    pub_uc_bound_.publish(uc_bound_mat);

    // Calculate mRPI for tube MPC
    NODELET_INFO_STREAM("[CHASER COORD]: Beginning mRPI calculation.");
    while(ros::ok() && !mrpi_finished_) {
      ros::spinOnce();
      loop_rate.sleep();
    }
    NODELET_INFO_STREAM("[CHASER COORD]: mRPI calculation finished.");
    use_tube_mpc_ = false;
    (use_tube_mpc_ == true) ? td_control_mode_ = "track_tube" : td_control_mode_ = "track";
    td_state_mode_ = "ekf";

    // Track the ref traj using tube MPC
    NODELET_INFO_STREAM("[CHASER COORD]: Beginning gain translation unit test.");
    while (ros::ok() && !traj_finished_) {
      ros::spinOnce();
      loop_rate.sleep();
    }

    test_finished_ = true;
    td_control_mode_ = "regulate";
    td_gain_mode_ = 0;
    return 1;
  }

  int test16() {
    /* "MIT" test trajectory like test15 but with tube MPC
    */

    // sample uc_bound
    MatrixXf eig_uc_bound_mat(4, 3);
    eig_uc_bound_mat << 0.005, 0.005, 0.005,
                        0.005, 0.005, 0.005,
                        0.0025, 0.0025, 0.0025,
                        -0.0025, -0.0025, 0.0025;  // +pos; -pos; +vel; -vel, per control_dt

    // mRPI trigger
    std_msgs::Float64MultiArray uc_bound_mat;
    tf::matrixEigenToMsg(eig_uc_bound_mat, uc_bound_mat);

    ros::Rate loop_rate(10.0);

    // BEGIN TEST
    // Read in the trajectory
    read_traj_standard_format("caroline", 0);
    send_body_traj();

    pub_uc_bound_.publish(uc_bound_mat);

    // Calculate mRPI for tube MPC
    NODELET_INFO_STREAM("[CHASER COORD]: Beginning mRPI calculation.");
    while(ros::ok() && !mrpi_finished_) {
      ros::spinOnce();
      loop_rate.sleep();
    }
    NODELET_INFO_STREAM("[CHASER COORD]: mRPI calculation finished.");
    use_tube_mpc_ = true;
    (use_tube_mpc_ == true) ? td_control_mode_ = "track_tube" : td_control_mode_ = "track";
    td_state_mode_ = "ekf";

    // Track the ref traj using tube MPC
    NODELET_INFO_STREAM("[CHASER COORD]: Beginning gain translation unit test.");
    while (ros::ok() && !traj_finished_) {
      ros::spinOnce();
      loop_rate.sleep();
    }

    test_finished_ = true;
    td_control_mode_ = "regulate";
    td_gain_mode_ = 0;
    return 1;
  }

  int debug_traj_body() {
    /* Debug sending out traj_body. (test77)
    */
    NODELET_INFO_STREAM("[CHASER COORD]: Performing debug test.");

    // sample uc_bound
    MatrixXf eig_uc_bound_mat(4, 3);
    eig_uc_bound_mat << 0.005, 0.005, 0.005,
                        0.005, 0.005, 0.005,
                        0.0025, 0.0025, 0.0025,
                        -0.0025, -0.0025, 0.0025;  // +pos; -pos; +vel; -vel, per control_dt;

    // mRPI trigger
    std_msgs::Float64MultiArray uc_bound_mat;
    tf::matrixEigenToMsg(eig_uc_bound_mat, uc_bound_mat);

    ros::Rate loop_rate(10.0);

    q_targ0_ << 0.0, 0.0, 0.0, 1.0;
    omega_targ0_ << 0.0, 0.0, 0.05;

    // Read in the trajectory
    read_traj_standard_format("caroline", 0);
    send_body_traj();

    // pub_uc_bound_.publish(uc_bound_mat);
    //
    // // Calculate mRPI for tube MPC
    // NODELET_INFO_STREAM("[CHASER COORD]: Beginning mRPI calculation.");
    // while(ros::ok() && !mrpi_finished_) {
    //   ros::spinOnce();
    //   loop_rate.sleep();
    // }
    // NODELET_INFO_STREAM("[CHASER COORD]: mRPI calculation finished.");
    //

    // dummy update
    q_targ0_ << 0.0, 0.0, 0.0, 1.0;
    omega_targ0_ << 0.0, 0.0, 0.075;

    geometry_msgs::TwistWithCovariance p;
    p.twist.angular.x = omega_targ0_(0);
    p.twist.angular.y = omega_targ0_(1);
    p.twist.angular.z = omega_targ0_(2);
    target_est_twist_pub_.publish(p);

    geometry_msgs::PoseWithCovariance t;
    t.pose.position.x = 0.0;
    t.pose.position.y = 0.0;
    t.pose.position.z = 0.0;
    t.pose.orientation.x = q_targ0_(0);
    t.pose.orientation.y = q_targ0_(1);
    t.pose.orientation.z = q_targ0_(2);
    t.pose.orientation.w = q_targ0_(3);
    target_est_pose_pub_.publish(t);

    ros::Duration(2.0).sleep();
    use_tube_mpc_ = false;
    td_state_mode_ = "ekf";
    (use_tube_mpc_ == true) ? td_control_mode_ = "track_tube" : td_control_mode_ = "track";

    // Track the ref traj using MPC
    NODELET_INFO_STREAM("[CHASER COORD]: Beginning traj track...");
    while (ros::ok() && !traj_finished_) {
      ros::spinOnce();
      loop_rate.sleep();
    }

    test_finished_ = true;
    td_control_mode_ = "regulate";
    return 1;
  }

  void check_regulate() {
    // Spin until regulation is complete (within threshold)
    ros::Rate loop_rate(10.0);  // Hz
    NODELET_INFO_STREAM("[CHASER COORDINATOR]: Beginning regulation.");
    auto regulate_start = std::chrono::high_resolution_clock::now();
    while (ros::ok() && !chaser_regulate_finished_) {
      /*
      double error_x = std::abs(x_real_complete_(0) - x0_(0));
      double error_y = std::abs(x_real_complete_(1) - x0_(1));
      double error_z = std::abs(x_real_complete_(2) - x0_(2));
      double error_qx = std::abs(x_real_complete_(3) - a0_(0));
      double error_qy = std::abs(x_real_complete_(4) - a0_(1));
      double error_qz = std::abs(x_real_complete_(5) - a0_(2));
      double error_qw = std::abs(x_real_complete_(6) - a0_(3));
      double error_qx_neg = std::abs(x_real_complete_(3) - (-1)*a0_(0));
      double error_qy_neg = std::abs(x_real_complete_(4) - (-1)*a0_(1));
      double error_qz_neg = std::abs(x_real_complete_(5) - (-1)*a0_(2));
      double error_qw_neg = std::abs(x_real_complete_(6) - (-1)*a0_(3));
      double error_vx = std::abs(x_real_complete_(7));
      double error_vy = std::abs(x_real_complete_(8));
      double error_vz = std::abs(x_real_complete_(9));
      double error_wx = std::abs(x_real_complete_(10));
      double error_wy = std::abs(x_real_complete_(11));
      double error_wz = std::abs(x_real_complete_(12));

      double error_q_sum = error_qx + error_qy + error_qz + error_qw;
      double error_q_sum_neg = error_qx_neg + error_qy_neg + error_qz_neg + error_qw_neg;
      if (error_q_sum_neg < error_q_sum) {
        error_qx = error_qx_neg;
        error_qy = error_qy_neg;
        error_qz = error_qz_neg;
        error_qw = error_qw_neg;
      }

      if (ground_.compare("true") == 0) {
        if (error_x < pos_reg_thresh_ && error_y && pos_reg_thresh_ && error_qx < att_reg_thresh_ && error_qy < att_reg_thresh_ && error_qz < att_reg_thresh_ && error_qw < att_reg_thresh_ &&
            error_vx < vel_reg_thresh_ && error_vy < vel_reg_thresh_ && error_wz < omega_reg_thresh_) {
          chaser_regulate_finished_ = true;
          NODELET_INFO_STREAM("[CHASER COORDINATOR]: Regulation complete.");
        }
      }
      else {
        if (error_x < pos_reg_thresh_ && error_y && pos_reg_thresh_ && error_z < pos_reg_thresh_ && error_qx < att_reg_thresh_ && error_qy < att_reg_thresh_ && error_qz < att_reg_thresh_ && error_qw < att_reg_thresh_ &&
            error_vx < vel_reg_thresh_ && error_vy < vel_reg_thresh_ && error_vz < vel_reg_thresh_ && error_wx < omega_reg_thresh_ && error_wy < omega_reg_thresh_ && error_wz < omega_reg_thresh_) {
          chaser_regulate_finished_ = true;
          NODELET_INFO_STREAM("[CHASER COORDINATOR]: Regulation complete.");
        }
      }
      */

      auto regulate_time = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> regulate_elapsed = regulate_time - regulate_start;
      while (ros::ok() && std::chrono::duration<double>(regulate_elapsed).count() < reg_time_) {
        loop_rate.sleep();
        regulate_time = std::chrono::high_resolution_clock::now();
        regulate_elapsed = regulate_time - regulate_start;
      }

      chaser_regulate_finished_ = true;
      std::cout << "Chaser regulation time: " << ros::Time::now().toSec() << std::endl;
      NODELET_INFO_STREAM("[CHASER COORDINATOR]: Regulation complete.");

      //ros::spinOnce();
      //loop_rate.sleep();
    }
  }

  void PublishTDStatus(const ros::TimerEvent&) {
    /* Main coordinator of nodelet logic.
    */
    trace_msgs::TDStatus msg;
    msg.stamp = ros::Time::now();
    msg.test_number = test_number_;
    msg.test_LUT = test_LUT_;
    msg.test_tumble_type = test_tumble_type_;
    msg.test_control_mode = test_control_mode_;
    msg.test_state_mode = test_state_mode_;
    msg.LUT_param = LUT_param_;
    msg.chaser_coord_ok = chaser_coord_ok_;
    msg.slam_activate = slam_activate_;
    msg.motion_planner_interface_activate = motion_planner_interface_activate_;
    msg.uc_bound_activate = uc_bound_activate_;
    msg.chaser_regulate_finished = chaser_regulate_finished_;
    msg.slam_converged = slam_converged_;
    msg.inertia_estimated = inertia_estimated_;
    msg.motion_plan_finished = motion_plan_finished_;
    msg.motion_plan_wait_time = motion_plan_wait_time_;
    msg.uc_bound_finished = uc_bound_finished_;
    msg.mrpi_finished = mrpi_finished_;
    msg.traj_finished = traj_finished_;
    msg.test_finished = test_finished_;
    msg.default_control = default_control_;
    msg.td_control_mode = td_control_mode_;
    msg.td_state_mode = td_state_mode_;
    msg.td_flight_mode = td_flight_mode_;
    msg.casadi_on_target = false;
    msg.test_slam_spoof = test_slam_spoof_;
    msg.gain_mode = td_gain_mode_;
    msg.online_update_mode = online_update_mode_;
    pub_status_.publish(msg);

    try {
      if (slam_activate_ == true && signal_sent_.slam_activate == 0) {
        signal_light_state.state = ff_msgs::SignalState::CHARGING;
        pub_signal_.publish(signal_light_state);
        signal_sent_.slam_activate = 1;
      }
      if (motion_planner_interface_activate_ == true && signal_sent_.motion_planner_interface_activate == 0) {
        signal_light_state.state = ff_msgs::SignalState::SUCCESS;
        pub_signal_.publish(signal_light_state);
        signal_sent_.motion_planner_interface_activate = 1;
      }
      if ( (td_control_mode_.compare("track") == 0 || td_control_mode_.compare("track_tube") == 0)
        && signal_sent_.control_mode_activate == 0){
        signal_light_state.state = ff_msgs::SignalState::CLEAR;
        pub_signal_.publish(signal_light_state);
        signal_sent_.control_mode_activate = 1;
      }
      if (test_finished_ == true && signal_sent_.test_finished == 0) {
        signal_light_state.state = ff_msgs::SignalState::ENTER_HATCHWAY;
        pub_signal_.publish(signal_light_state);
        signal_sent_.test_finished = 1;
      }
    }
    catch (...) { }
  }

  void test_num_callback(const trace_msgs::TDTestNumber::ConstPtr msg) {
    test_number_ = msg->test_number;
    if (test_number_ == -1) {
      // Re-enable default controller
      enable_default_ctl();

      // Set flight mode to off
      td_flight_mode_ = "off";
      if (!ff_util::FlightUtil::GetFlightMode(flight_mode_, td_flight_mode_)) {
          return;
      }
      pub_flight_mode_.publish(flight_mode_);

      trace_msgs::TDStatus msg;
      msg.stamp = ros::Time::now();
      msg.test_number = test_number_;
      msg.chaser_coord_ok = chaser_coord_ok_;
      msg.slam_activate = slam_activate_;
      msg.motion_planner_interface_activate = motion_planner_interface_activate_;
      msg.uc_bound_activate = uc_bound_activate_;
      msg.chaser_regulate_finished = chaser_regulate_finished_;
      msg.slam_converged = slam_converged_;
      msg.inertia_estimated = inertia_estimated_;
      msg.motion_plan_finished = motion_plan_finished_;
      msg.motion_plan_wait_time = motion_plan_wait_time_;
      msg.uc_bound_finished = uc_bound_finished_;
      msg.mrpi_finished = mrpi_finished_;
      msg.traj_finished = traj_finished_;
      msg.test_finished = test_finished_;
      msg.default_control = default_control_;
      msg.td_control_mode = td_control_mode_;
      msg.td_state_mode = td_state_mode_;
      msg.td_flight_mode = td_flight_mode_;
      msg.casadi_on_target = false;
      msg.test_slam_spoof = test_slam_spoof_;
      pub_status_.publish(msg);

    }
  }

  void motion_planner_interface_status_callback(const trace_msgs::TDMotionPlannerInterfaceStatus::ConstPtr msg) {
    motion_plan_finished_ = msg->motion_plan_finished;
  }

  void uc_bound_status_callback(const trace_msgs::TDUCBoundStatus::ConstPtr msg) {
    uc_bound_finished_ = msg->uc_bound_finished;
    uc_bound_unit_test_complete_ = msg->unit_test_complete;
  }

  void casadi_status_callback(const trace_msgs::TDCasadiStatus::ConstPtr msg) {
    chaser_coord_ok_ = msg->chaser_coord_ok;
    mrpi_finished_ = msg->mrpi_finished;
    traj_finished_ = msg->traj_finished;
  }

  void slam_info_callback(const trace_msgs::TDSlamInfo::ConstPtr msg) {
    if (!test_slam_spoof_) {
      slam_converged_ = msg->converged;
      inertia_estimated_ = msg->inertia_estimated;
    }
  }

  void flight_mode_callback(const ff_msgs::FlightMode::ConstPtr msg) {
    // pancakes for will
    flight_mode_check_ = msg->name;
  }

  void inertia_callback(const geometry_msgs::Inertia::ConstPtr inertia_msg) {
    J_(0,0) = inertia_msg->ixx;
    J_(0,1) = inertia_msg->ixy;
    J_(0,2) = inertia_msg->ixz;
    J_(1,1) = inertia_msg->iyy;
    J_(1,2) = inertia_msg->iyz;
    J_(2,2) = inertia_msg->izz;
    J_(1,0) = J_(0,1);
    J_(2,1) = J_(1,2);
    J_(2,0) = J_(0,2);
  }

  void gnc_ctl_setpoint_callback(const ros::TimerEvent& ) {
    /*
    Send out gnc_ctl_setpoints periodically, if activated

    TODO: for now it only regulates to a stationary start point.
    */
    if (ACTIVATE_DEFAULT_REGULATION_ == 1) {
      // set up the trajectory publishing
      geometry_msgs::Point traj_pos;
      geometry_msgs::Vector3 traj_vel;
      geometry_msgs::Quaternion traj_quat;
      geometry_msgs::Vector3 traj_omega;

      // position
      traj_pos.x = x0_(0);
      traj_pos.y = x0_(1);
      traj_pos.z = x0_(2);

      // velocity
      traj_vel.x = 0.0;
      traj_vel.y = 0.0;
      traj_vel.z = 0.0;

      // attitude
      traj_quat.x = a0_(0);;
      traj_quat.y = a0_(1);
      traj_quat.z = a0_(2);
      traj_quat.w = a0_(3);

      // angular rate
      traj_omega.x = 0.0;
      traj_omega.y = 0.0;
      traj_omega.z = 0.0;

      // desired acceleration is 0
      geometry_msgs::Vector3 traj_accel;
      traj_accel.x = 0.0;
      traj_accel.y = 0.0;
      traj_accel.z = 0.0;

      // desired angular acceleration is 0 (will take a little while to get to smooth, constant w)
      geometry_msgs::Vector3 traj_alpha;
      traj_alpha.x = 0.0;
      traj_alpha.y = 0.0;
      traj_alpha.z = 0.0;

      // Publish the trajectory topic
      ff_msgs::ControlState new_state;
      new_state.when = ros::Time::now();;
      new_state.pose.position = traj_pos;
      new_state.pose.orientation = traj_quat;
      new_state.twist.linear = traj_vel;
      new_state.twist.angular = traj_omega;
      new_state.accel.linear = traj_accel;
      new_state.accel.angular = traj_alpha;

      pub_traj_default_.publish(new_state);
    }
  }

  /* ************************************************************************** */
  void slam_targ_att_callback(const geometry_msgs::PoseWithCovariance::ConstPtr msg) {
    /* Updates Target pose estimate using SLAM. If SLAM spoof is on, this data is known exactly.
    */
    q_targ_(0) = msg->pose.orientation.x;
    q_targ_(1) = msg->pose.orientation.y;
    q_targ_(2) = msg->pose.orientation.z;
    q_targ_(3) = msg->pose.orientation.w;
  }

  /* ************************************************************************** */
  void slam_targ_omega_callback(const geometry_msgs::TwistWithCovariance::ConstPtr msg) {
    /* Updates Target angular velocity estimate using SLAM. If SLAM spoof is on, this data is known exactly.
    */
    omega_targ_(0) = msg->twist.angular.x;
    omega_targ_(1) = msg->twist.angular.y;
    omega_targ_(2) = msg->twist.angular.z;
  }

  void targ_att0_callback(const geometry_msgs::PoseWithCovariance::ConstPtr& msg) {
    /* Callback for motion planner inputs.
    */
    q_targ0_(0) = msg->pose.orientation.x;
    q_targ0_(1) = msg->pose.orientation.y;
    q_targ0_(2) = msg->pose.orientation.z;
    q_targ0_(3) = msg->pose.orientation.w;
  }

  void targ_omega0_callback(const geometry_msgs::TwistWithCovariance::ConstPtr& msg) {
    /* Callback for motion planner inputs.
    */
    omega_targ0_(0) = msg->twist.angular.x;
    omega_targ0_(1) = msg->twist.angular.y;
    omega_targ0_(2) = msg->twist.angular.z;
  }

  void ekf_callback(const ff_msgs::EkfState::ConstPtr msg) {
      /*
      The `gnc/ekf` subscriber callback
      Called at 62.5 Hz
      */
      float qx = msg->pose.orientation.x;
      float qy = msg->pose.orientation.y;
      float qz = msg->pose.orientation.z;
      float qw = msg->pose.orientation.w;

      if (qx != 0 || qy != 0 || qz != 0 || qw != 0) {
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
      }
  }

  ///
  /// Support functions
  ///
  MatrixXd read_traj_CSV(){
    /*
    Read in a trajecotry in CSV format
    */
    ros::param::getCached("/td/chaser_traj_file", traj_filename_);  // filename to read in
    MatrixXd input_data = csv::load_csv<MatrixXd>(traj_filename_);  // read in CSV trajectory

    return input_data;
  }

  std::tuple<MatrixXd, int> read_traj_standard_format(std::string format, int transform){
    /*
    Read in a trajectory in standard format:
    [t, position, velocity, linear accel, jerk]

    Params:
    traj_filename_: the full path to the trajectory directory of interest

    Inputs:
    format: "roberto" for old format, "caroline" for new standard format
    transform: {0, 1} if a transformation from Target to inertial frame is needed

    Outputs:
    tuple of output_x Eigen::MatrixXd and and int for number of setpoints.
    Automatically publishes to controller
    */

    ros::param::getCached("/td/chaser_traj_file", traj_filename_);  // filename to read in

    int num_setpoints;
    MatrixXd output_x;
    MatrixXd output_u;

    if (format == "roberto") {
      // manually
      // std::string DATA_PATH = ros::package::getPath("data")+"/";
      // traj_filename_ = DATA_PATH + "input/DLR_chaser_LUT_4_20_20/DATA/";

      // Either choose trajectory from motion planner or from predetermined trajectory file
      //TODO: this format is deprecated!
      output_x = csv::load_dat_chaser<MatrixXd>(traj_filename_);  // read in roberto format
      num_setpoints = output_x.rows();
    }
    else if (format == "caroline"){
      // Either choose trajectory from motion planner or from predetermined trajectory file
      output_x = traj_utils::get_planner_output_x<MatrixXd>(traj_filename_);
      output_u = traj_utils::get_planner_output_u<MatrixXd>(traj_filename_);
      num_setpoints = output_x.rows();
    }

    if (transform == 1){
      resolve_x_des_traj_T_to_I();  // convert to inertial coordinates
    }
    eigen_x_des_traj_ = output_x;

    // translate from TVR frame to ISS frame (don't do this for tracking tests already in ISS coordinates!)
    if (ground_.compare("true") != 0 && test_number_ != 3 && test_number_ != 4
      && test_number_ != 15 && test_number_ != 16) {
      for (int i = 0; i < num_setpoints; i++) {
        // adjust y for standard format TVR convention
        double extra_offset = 0.0;
        if (test_number_ > 100) {
          std::string test_number_str = std::to_string(test_number_);
          if (test_number_str[0] == '1') { // x
            extra_offset = -1.5;
          }
          if (test_number_str[0] == '3') { // z
            extra_offset = -1.5;
          }
        }
        eigen_x_des_traj_(i,1) = eigen_x_des_traj_(i,1) + r_RI_(0);
        eigen_x_des_traj_(i,2) = eigen_x_des_traj_(i,2) + r_RI_(1) + extra_offset;
        eigen_x_des_traj_(i,3) = eigen_x_des_traj_(i,3) + r_RI_(2);
      }
    }

    send_traj_to_controller();  // send the full trajectory off to any controllers waiting for it

    return std::make_tuple(output_x, num_setpoints);
  }

  void send_traj_to_controller(){
    /*
    MPC requires the full traj: send it via message
    */
    // IOFormat HeavyFmt(FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");
    std::string sep = "\n----------------------------------------\n";
    NODELET_DEBUG_STREAM("Sending trajectory to casadi_nmpc... ");
    // NODELET_DEBUG_STREAM(sep << eigen_x_des_traj_ << sep);

    std_msgs::Float64MultiArray eigen_x_des_traj_msg;
    tf::matrixEigenToMsg(eigen_x_des_traj_, eigen_x_des_traj_msg);

    pub_x_des_traj_.publish(eigen_x_des_traj_msg);
  }

  void send_body_traj() {
    std::cout << "[CHASER COORD]: SENDING TRAJECTORY..." << std::endl;
    /* Converts standard format nominal traj (in inertial frame) to nominal Target body frame and sends to controller.
    Note that the standard format trajectory begins AFTER a set wait time; this propagation starts after that wait time.
    */
    // the msg we care about filling
    trace_msgs::TDTrajBody eigen_traj_body_msg;

    // Get inertia tensor
    std::vector<float> J_vec;
    ros::param::getCached("/td/uc_bound/target_J", J_vec);
    int k = 0;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
          J_(i,j) = J_vec[k];
          k++;
      }
    }
    
    eigen_x_des_traj_body_ = eigen_x_des_traj_;  // this will also populate attitude with the nominal propagation
    MatrixXd eigen_x_des_traj_offset_ = eigen_x_des_traj_;
    int num_rows = eigen_x_des_traj_body_.rows();
    if (ground_.compare("true") == 0) {
        eigen_x_des_traj_body_(0, 3) = 0.0;
        eigen_x_des_traj_body_(0, 6) = 0.0;
      }
    
    eigen_x_des_traj_offset_.col(1) = eigen_x_des_traj_.col(1) - targ_offset_(0) * VectorXd::Ones(num_rows);
    eigen_x_des_traj_offset_.col(2) = eigen_x_des_traj_.col(2) - targ_offset_(1) * VectorXd::Ones(num_rows);
    eigen_x_des_traj_offset_.col(3) = eigen_x_des_traj_.col(3) - targ_offset_(2) * VectorXd::Ones(num_rows);
    Eigen::Vector3d original_pos0 = eigen_x_des_traj_offset_.block(0, 1, 1, 3).transpose();
    Eigen::Vector3d original_vel0 = eigen_x_des_traj_offset_.block(0, 4, 1, 3).transpose();
    
    // target attitude, nominal
    tf2::Quaternion q_targ_IB{q_targ_(0), q_targ_(1), q_targ_(2), q_targ_(3)};  // target body pose wrt inertial frame
    Eigen::Matrix3d R_targ_IB = q2dcm(q_targ_).cast<double>();
    
    // target velocity, nominal
    Eigen::Vector3d omega_targ_I = R_targ_IB * omega_targ0_.cast<double>();
    
    // NODELET_ERROR_STREAM("J " << J_ << "\R_targ_IB" << R_targ_IB << "\nomega_targ_I" << omega_targ_I <<
    //   "\noriginal_pos0" << original_pos0 << "\noriginal_vel0" << original_vel0);
    
    // Propagate for all time steps
    double dt_traj = eigen_x_des_traj_(1, 0) - eigen_x_des_traj_(0, 0);
    double t = 0.0;
    state_type x;  // eigen [1 x 7] of [quat, orientation]
    
    // we have these initial conditions from ROS params
    x.segment(0,4) = q_targ_;  // this is the q used by the motion planner at the instant it starts (after wait time)
    x.segment(4,3) = omega_targ_;
    // NODELET_ERROR_STREAM("At body send, quat is: " << q_targ_);
    for (int i = 0; i < num_rows; i++) {
        Eigen::Vector3d original_pos = eigen_x_des_traj_offset_.block(i, 1, 1, 3).transpose();
        Eigen::Vector3d original_vel = eigen_x_des_traj_offset_.block(i, 4, 1, 3).transpose();

        // Position
        eigen_x_des_traj_body_.block(i, 1, 1, 3) = (R_targ_IB.transpose() * original_pos).transpose();

        // Velocity
        eigen_x_des_traj_body_.block(i, 4, 1, 3) = (R_targ_IB.transpose()*(original_vel - omega_targ_I.cross(original_pos))).transpose();
        if (ground_.compare("true") == 0) {
                eigen_x_des_traj_body_(i, 3) = 0.0;
                eigen_x_des_traj_body_(i, 6) = 0.0;
        }

        // Quaternion (B wrt I, R_IB)
        geometry_msgs::Quaternion q_targ_IB_msg;
        q_targ_IB_msg = tf2::toMsg(q_targ_IB);
        eigen_traj_body_msg.q_targ_0_hist.push_back(q_targ_IB_msg);

        double t_next = t + dt_traj;
        if (ground_.compare("true") == 0) {
          integrate_adaptive(stepper_, dynamic_step_ground_{J_}, x, t, t_next, dt_traj);
        }
        else {
          integrate_adaptive(stepper_, dynamic_step_{J_}, x, t, t_next, dt_traj);
        }

        t += dt_traj;
        R_targ_IB = q2dcm(x.head(4)).cast<double>();
        q_targ_IB = tf2::Quaternion{x(0), x(1), x(2), x(3)};
        omega_targ_I = R_targ_IB * x.tail(3).transpose().cast<double>();
    }

    // populate the TrajBody msg
    std_msgs::Float64MultiArray eigen_x_des_traj_body_msg;
    tf::matrixEigenToMsg(eigen_x_des_traj_body_, eigen_x_des_traj_body_msg);
    eigen_traj_body_msg.traj_body = eigen_x_des_traj_body_msg;

    NODELET_DEBUG_STREAM("Body frame traj: ");
    // std::string sep = "\n----------------------------------------\n";
    // NODELET_DEBUG_STREAM(sep << eigen_x_des_traj_body_ << sep);

    pub_x_des_traj_body_.publish(eigen_traj_body_msg);

  }

  Eigen::Matrix3f q2dcm(const Vector4f &q)
  {
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

  void debug(){
    /*
    debug function call to test compatibility with other nodelets
    */
    NODELET_DEBUG_STREAM("debug called...");

    std::string RELATIVE_PATH = "/bin/ground-traj1-public/";
    std::string TRAJ_DIR_PATH = ros::package::getPath("motion_planner_interface");
    TRAJ_DIR_PATH += RELATIVE_PATH;
    ros::param::set("/td/chaser_traj_file", TRAJ_DIR_PATH);

    std::string format("caroline");
    NODELET_DEBUG_STREAM(traj_filename_);
    std::tie(eigen_x_des_traj_, Nf_) = read_traj_standard_format(format, 0);
    NODELET_DEBUG_STREAM(eigen_x_des_traj_);
    resolve_x_des_traj_T_to_I();
    send_traj_to_controller();

    // int t_idx = 0;
    // ff_msgs::ControlState new_state = create_ControlState_from_x_des(t_idx);
    // pub_traj_.publish(new_state);
  }

  void resolve_x_des_traj_T_to_I(){
    /*
    Convert the reference trajectory entirely to Chaser coordinates wrt the Inertial (ISS) frame.
    The initial offset is relative to the Target offset start point---this needs to be in ISS frame origin coordinates.
    */
    for (int idx = 0; idx < Nf_; idx++) {
      float x = eigen_x_des_traj_(idx, 1);  // chaser wrt chaser start frame @ t=0
      float y = eigen_x_des_traj_(idx, 2);
      float z = eigen_x_des_traj_(idx, 3);

      float x_TI, y_TI, z_TI;  // Target wrt Inertial, INERTIAL
      float x_CT, y_CT, z_CT;  // Chaser wrt Target, INERTIAL

      ros::param::getCached("/td/x_TI", x_TI);  // Target wrt Inertial, INERTIAL
      ros::param::getCached("/td/y_TI", y_TI);
      ros::param::getCached("/td/z_TI", z_TI);

      ros::param::getCached("/td/x_CT", x_CT);  // Chaser wrt Target, INERTIAL
      ros::param::getCached("/td/y_CT", y_CT);
      ros::param::getCached("/td/z_CT", z_CT);

      // need to convert to Inertial frame with correct offset...
      tf2::Vector3 xyz_TI(x_TI, y_TI, z_TI);  // Target, in Inertial frame
      tf2::Vector3 xyz_CT(x_CT, y_CT, z_CT);  // Chaser w.r.t. Target at holding point (@t=0), in Inertial frame
      tf2::Vector3 xyz_CI(x, y, z);           // Chaser w.r.t. holding point (@t=0), in Inertial frame

      xyz_CI = xyz_TI + xyz_CT + xyz_CI;  // Chaser in true Inertial frame (translated from Inertial holding point)

      eigen_x_des_traj_(idx, 1) = xyz_CI.m_floats[0];
      eigen_x_des_traj_(idx, 2) = xyz_CI.m_floats[1];
      eigen_x_des_traj_(idx, 3) = xyz_CI.m_floats[2];
    }
  }

  void disable_default_ctl() {
    /* Switch default controller off now.
    */
    NODELET_DEBUG_STREAM("[CHASER COORDINATOR]: Disabling default controller...");

    auto serv_start = std::chrono::high_resolution_clock::now();

    // Disable the default controller so tube-MPC can run
    std_srvs::SetBool srv;
    srv.request.data = false;
    serv_ctl_enable_.call(srv);

    default_control_ = false;

    auto serv_finish = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> serv_elapsed = serv_finish - serv_start;

    std::cout << "Controller disable service time: " << serv_elapsed.count() << " seconds."<< std::endl;

    //ff_msgs::ControlGoal goal;
    //goal.command = ff_msgs::ControlGoal::STOP;
    //client_control_.SendGoal(goal);
    //default_control_ = false;

    NODELET_DEBUG_STREAM("[CHASER COORDINATOR]: Ctl disable service result: " << srv.response.message);
  }

  void enable_default_ctl() {
    /* Switch default controller off now.
    */
    NODELET_DEBUG_STREAM("[CHASER COORDINATOR]: Enabling default controller...");

    // Disable the default controller so tube-MPC can run
    std_srvs::SetBool srv;
    srv.request.data = true;
    serv_ctl_enable_.call(srv);
    default_control_ = true;

    //ff_msgs::ControlGoal goal;
    //goal.command = ff_msgs::ControlGoal::STOP;
    //client_control_.SendGoal(goal);
    //default_control_ = true;

    NODELET_DEBUG_STREAM("[CHASER COORDINATOR]: Ctl enable service result: " << srv.response.message);
    //NODELET_DEBUG_STREAM("[CHASER COORDINATOR]: Default controller enabled.");
  }

  ff_msgs::ControlState create_ControlState_from_x_des(int t_idx){
    /* Using standard format x_des format, create a ControlState message
       Input:
       int t_idx: time index of eigen_x_des_traj_ to use
    */
      float t = eigen_x_des_traj_(t_idx, 0);  // data appears to be in inertial frame?
      NODELET_DEBUG_STREAM(t);
      float x = eigen_x_des_traj_(t_idx, 1);  // chaser wrt chaser frame @ t=0
      float y = eigen_x_des_traj_(t_idx, 2);
      float z = eigen_x_des_traj_(t_idx, 3);
      float xd = eigen_x_des_traj_(t_idx, 4);
      float yd = eigen_x_des_traj_(t_idx, 5);
      float zd = eigen_x_des_traj_(t_idx, 6);

      float qw = eigen_x_des_traj_(t_idx, 7);  // Chaser ang_vel in inertial frame
      float qx = eigen_x_des_traj_(t_idx, 8);  // Chaser att in inertial frame
      float qy = eigen_x_des_traj_(t_idx, 9);
      float qz = eigen_x_des_traj_(t_idx, 10);
      float wx = eigen_x_des_traj_(t_idx, 11);
      float wy = eigen_x_des_traj_(t_idx, 12);
      float wz = eigen_x_des_traj_(t_idx, 13);

      float xdd = eigen_x_des_traj_(t_idx, 14);  // Chaser accel in inertial frame
      float ydd = eigen_x_des_traj_(t_idx, 15);
      float zdd = eigen_x_des_traj_(t_idx, 16);
      float wxd = eigen_x_des_traj_(t_idx, 17);
      float wyd = eigen_x_des_traj_(t_idx, 18);
      float wzd = eigen_x_des_traj_(t_idx, 19);

      // set up the trajectory publishing message
      geometry_msgs::Point traj_pos;
      geometry_msgs::Vector3 traj_vel;
      geometry_msgs::Quaternion traj_quat;
      geometry_msgs::Vector3 traj_omega;

      // position
      traj_pos.x = x;
      traj_pos.y = y;
      traj_pos.z = z;

      // velocity
      traj_vel.x = xd;
      traj_vel.y = yd;
      traj_vel.z = zd;

      // attitude
      traj_quat.x = qx;
      traj_quat.y = qy;
      traj_quat.z = qz;
      traj_quat.w = qw;

      // point to Chaser

      // angular rate
      traj_omega.x = wx;
      traj_omega.y = wy;
      traj_omega.z = wz;

      // desired acceleration is 0
      geometry_msgs::Vector3 traj_accel;
      traj_accel.x = xdd;
      traj_accel.y = ydd;
      traj_accel.z = zdd;

      // desired angular acceleration is 0 (will take a little while to get to smooth, constant w)
      geometry_msgs::Vector3 traj_alpha;
      traj_alpha.x = wxd;
      traj_alpha.y = wyd;
      traj_alpha.z = wzd;

      if (ground_.compare("true") == 0) {  // zero out z-axis components
        traj_pos.z = -0.7;
        traj_vel.z = 0.0;

        // zero quat and normalize
        tf2::Quaternion q_gnd(0, 0, qz, qw);
        q_gnd.normalize();
        traj_quat.x = q_gnd[0];
        traj_quat.y = q_gnd[1];
        traj_quat.z = q_gnd[2];
        traj_quat.w = q_gnd[3];

        traj_omega.x = 0.0;
        traj_omega.y = 0.0;

        traj_accel.z = 0.0;
        traj_alpha.x = 0.0;
        traj_alpha.y = 0.0;
      }

      // Create the trajectory setpoint message
      ff_msgs::ControlState new_state;
      new_state.when = ros::Time::now();;
      new_state.pose.position = traj_pos;
      new_state.pose.orientation = traj_quat;
      new_state.twist.linear = traj_vel;
      new_state.twist.angular = traj_omega;
      new_state.accel.linear = traj_accel;
      new_state.accel.angular = traj_alpha;

      return new_state;
  }

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
};  // end class ChaserCoordinatorNodelet
}  // end namespace chaser_coordinator

// Declare the plugin
PLUGINLIB_EXPORT_CLASS(chaser_coordinator::ChaserCoordinatorNodelet, nodelet::Nodelet);
