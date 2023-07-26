/**
 * @file target_coordinator_nodelet.cc
 * @author Charles Oestreich, Keenan albee
 * @brief Nodelet for controlling target satellite to follow tumbling trajectories (for simulation purposes).
 * @version 0.1
 * @date 2023-04-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#define _USE_MATH_DEFINES

#include <math.h>
#include <string.h>
#include <Eigen/Dense>
#include <boost/numeric/odeint/algebra/vector_space_algebra.hpp>
#include <boost/numeric/odeint/integrate/integrate.hpp>
#include <boost/numeric/odeint/stepper/runge_kutta_dopri5.hpp>
#include <fstream>
#include <string>
#include <thread>

// FSW
#include <ff_util/ff_flight.h>
#include <ff_util/ff_nodelet.h>

#include <ff_msgs/ControlState.h>
#include <ff_msgs/EkfState.h>
#include <ff_msgs/FlightMode.h>

// ROS
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/package.h>
#include <std_srvs/SetBool.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>

// TRACE
#include <trace_msgs/TDStatus.h>
#include <trace_msgs/TDTestNumber.h>

#include "data/eigen_kdl.h"
#include "data/eigen_msg.h"
#include "data/att_utils.h"
#include "data/csv_read.h"


float EPSILON = std::numeric_limits<float>::epsilon();

using namespace Eigen;
using namespace boost::numeric::odeint;

namespace target_coordinator {

class TargetCoordinatorNodelet : public ff_util::FreeFlyerNodelet {
 public:
  TargetCoordinatorNodelet()
      : ff_util::FreeFlyerNodelet(true) {
  }  // don't need to define in ff_names.h, but can if desired
  ~TargetCoordinatorNodelet() {}

 private:
    // Publishers for setpoints, setpoints (unit test comparison), and flight mode
    ros::Publisher pub_gnc_ctl_setpoint_;
    ros::Publisher pub_flight_mode_;
    ros::Publisher pub_status_;
    ros::Publisher pub_x_des_traj_;

    // Subscribers for simulation truth pose and twist (unit test comparison)
    ros::Subscriber sub_ekf_;
    ros::Subscriber sub_test_number_;

    ros::Timer pub_status_timer_;

    int test_number_ = 0;
    bool target_coord_ok_ = true;
    bool target_regulate_finished_ = false;
    bool traj_finished_ = false;
    bool default_control_ = false;
    bool test_finished_ = false;
    std::string td_control_mode_ = "inactive";
    std::string td_state_mode_ = "ekf";
    std::string td_flight_mode_ = "nominal";
    bool casadi_on_target_ = true;
    int td_gain_mode_ = 0;

    std::string traj_filename_;
    ff_msgs::FlightMode flight_mode_;
    std::shared_ptr<std::thread> thread_;

    ros::ServiceClient serv_ctl_enable_;

    // vectors to hold target EKF state
    Vector3f p_truth_;
    Vector3f v_truth_;
    Vector4f q_truth_;
    Vector3f w_truth_;

    MatrixXd eigen_x_des_traj_;

    // inertia tensor
    Matrix3f J_;
    std::vector<float> J_vec_;

    // length of time before target tumble is considered settled
    int tumble_settle_length_;

    // Set type for state vector
    typedef Matrix<float, 1, 7> state_type;

    // Input trajectory data
    MatrixXd input_data_;

    // for accepting in the initial attitude/angular velocity
    Vector3f x0_;
    Vector4f a0_;
    Vector3f w0_;

    // extra parameters
    std::string ground_ = "false";
    std::string sim_ = "false";

    // regulation thresholds
    float pos_reg_thresh_;
    float vel_reg_thresh_;
    float att_reg_thresh_;
    float omega_reg_thresh_;

    float reg_time_;  // time to regulate
    float after_reg_pause_;  // pause length after regulation and before test.
    float max_omega_;  // max omega for setpoints

    // dynamic propagator stuff
    class dynamic_step_ {
      Matrix3f J_param;

     public:
      dynamic_step_(Matrix3f G) : J_param(G) {}

      void operator()(state_type const&x, state_type&dx, const float &t) const {
        VectorXf q = x.head(4);
        Vector3f w = x.segment(4, 3);
        Vector3f M = VectorXf::Zero(3);   // Assuming no external torques on target
        Matrix3f J_inv;

        J_inv = J_param.inverse();

        // Quaternion kinematics
        MatrixXf B(4, 4);
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
      dynamic_step_ground_(Matrix3f G) : J_param(G) {}

      void operator()(state_type const&x, state_type&dx, const float &t) const {
        VectorXf q = x.head(4);
        Vector3f w = x.segment(4, 3);

        // Quaternion kinematics
        MatrixXf B(4, 4);
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


  // This is called when the nodelet is loaded into the nodelet manager
  void Initialize(ros::NodeHandle *nh) {
    pub_status_timer_ = nh->createTimer(ros::Duration(0.2), &TargetCoordinatorNodelet::PublishTDStatus, this);

    ros::param::getCached("/td/sim", sim_);
    if (casadi_on_target_) {  // toggle default_control_ on or off
      default_control_ = false;
    }

    // Initialize ground truth subscribers
    sub_ekf_ = nh->subscribe<ff_msgs::EkfState>("gnc/ekf", 5, &TargetCoordinatorNodelet::ekfCallback, this);
    sub_test_number_ = nh->subscribe<trace_msgs::TDTestNumber>(
        "td/test_number", 5, &TargetCoordinatorNodelet::testNumberCallback,
        this);

    // Initialize publishers
    pub_flight_mode_ = nh->advertise<ff_msgs::FlightMode>(TOPIC_MOBILITY_FLIGHT_MODE, 1, true);  // FlightMode
    pub_gnc_ctl_setpoint_ = nh->advertise<ff_msgs::ControlState>(TOPIC_GNC_CTL_SETPOINT, 5, true);  // setpoints
    pub_status_ = nh->advertise<trace_msgs::TDStatus>("td/status", 5, true);
    pub_x_des_traj_ = nh->advertise<std_msgs::Float64MultiArray>(
        "td/tube_mpc/traj", 5, true);  // full traj---used once for TubeMPC

    // services
    serv_ctl_enable_ = nh->serviceClient<std_srvs::SetBool>(SERVICE_GNC_CTL_ENABLE);

    // Get inertia tensor from ROS (default is ENVISAT)
    ros::param::getCached("/td/target_coordinator/target_J", J_vec_);
    int k = 0;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        J_(i, j) = J_vec_[k];
        k++;
      }
    }

    ros::param::getCached("/td/target_coordinator/reg_time", reg_time_);
    ros::param::getCached("/td/target_coordinator/max_omega", max_omega_);  // Max omega for setpoints
    ros::param::getCached("/td/ground", ground_);
    ros::param::getCached("/td/target/after_reg_pause", after_reg_pause_);
    ros::param::getCached(
        "/td/target_coordinator/target_traj_file",
        traj_filename_);  // filename to read in (determines tumble type)
    NODELET_INFO_STREAM("[TARG COORD] Initialized.");
    thread_.reset(new std::thread(&target_coordinator::TargetCoordinatorNodelet::Run, this));
  }

  void Run() {
    ros::Rate loop_rate(10.0);  // Hz

    // wait for test number
    while (ros::ok() && test_number_ == 0) {
      ros::spinOnce();
      loop_rate.sleep();
    }

    process_test_number();  // get any parameter settings for Target

    // skip turning on impellers for tests that don't use Target
    if (test_number_ != 1 && test_number_ != 3 && test_number_ != 4 && test_number_ != 5 && test_number_ != 12) {
      input_data_ = csv::load_dat_target<MatrixXd>(traj_filename_);
      a0_(3) = input_data_(0, 7);
      a0_(0) = input_data_(0, 8);
      a0_(1) = input_data_(0, 9);
      a0_(2) = input_data_(0, 10);
      w0_(0) = input_data_(0, 11);
      w0_(1) = input_data_(0, 12);
      w0_(2) = input_data_(0, 13);
      std::cout << "Target: a0: " << std::endl << a0_ << std::endl;
      std::cout << "Target: w0: " << std::endl << w0_ << std::endl;
      if (!ff_util::FlightUtil::GetFlightMode(flight_mode_, td_flight_mode_)) {
          return;
      }  // create a nominal FlightMode
      pub_flight_mode_.publish(flight_mode_);
    } else {
      if (!ff_util::FlightUtil::GetFlightMode(flight_mode_, "off")) {
          return;
      }  // create a nominal FlightMode
      pub_flight_mode_.publish(flight_mode_);
    }

    ros::Duration(2.0).sleep();  // wait for params to be set
    get_params();

    if (test_number_ != 1) {
      // Regulate to starting point
      NODELET_INFO_STREAM("[TARG COORD] Beginning regulation.");
      auto regulate_start = std::chrono::high_resolution_clock::now();

      if (!default_control_) {  // use CasADi: regulate and turn off default controller
        td_control_mode_ = "regulate";
        td_state_mode_ = "ekf";
        ros::Duration(0.4)
            .sleep();  // make sure casadi_nmpc gets the regulate/ekf settings
                       // before disabling default controller.
        disable_default_ctl();
      }

      // regulate
      while (ros::ok() && !target_regulate_finished_) {
        auto regulate_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> regulate_elapsed = regulate_time - regulate_start;

        while (ros::ok() && std::chrono::duration<double>(regulate_elapsed).count() < reg_time_) {
          loop_rate.sleep();
          regulate_time = std::chrono::high_resolution_clock::now();
          regulate_elapsed = regulate_time - regulate_start;
          if (default_control_) {  // send a regulate command
            send_default_regulate_cmd();
          }
        }
        target_regulate_finished_ = true;
        // check_regulate();
        loop_rate.sleep();
      }
    }

    NODELET_INFO_STREAM("[TARG COORD] Regulation complete.");
    NODELET_INFO_STREAM("[TARG COORD] Target regulation time from startup: " << ros::Time::now().toSec() << std::endl);
    ros::Duration(after_reg_pause_).sleep();  // pause between regulation and beginning of test

    while (ros::ok() && !test_finished_) {
      // skip non-Target tests
      if (test_number_ != 1 && test_number_ != 3 && test_number_ != 4 && test_number_ != 5 && test_number_ != -1) {
        // Unit test
        NODELET_DEBUG_STREAM("[TARG COORD] Running tumble.");
        if (traj_filename_.empty()) {
          target_coord_ok_ = false;
        } else {
          tumble();  // follows the traj file's tumble
        }
      }

      ros::spinOnce();
      loop_rate.sleep();
    }
  }

  void process_test_number() {
    /* Determine any parameter settings
    */
    std::string test_number_str = std::to_string(test_number_);

    // Test number format is xx(xxxxxx); not unit tests
    if (test_number_ > 100) {
      if (test_number_str[5] == '1') {  // use CasADi
        default_control_ = false;
        casadi_on_target_ = true;
      } else if (test_number_str[5] == '2') {  // use default PD
        default_control_ = true;
        casadi_on_target_ = false;
      }
    }
  }

  void check_regulate() {
    /* Performs a norm check to complete regulation
    TODO: deprecated!
    */
    if (default_control_) {
      send_default_regulate_cmd();
    }

    double error_x = std::abs(p_truth_(0) - x0_(0));
    double error_y = std::abs(p_truth_(1) - x0_(1));
    double error_z = std::abs(p_truth_(2) - x0_(2));
    double error_qx = std::abs(q_truth_(0) - a0_(0));
    double error_qy = std::abs(q_truth_(1) - a0_(1));
    double error_qz = std::abs(q_truth_(2) - a0_(2));
    double error_qw = std::abs(q_truth_(3) - a0_(3));
    double error_qx_neg = std::abs(q_truth_(3) - (-1)*a0_(0));
    double error_qy_neg = std::abs(q_truth_(4) - (-1)*a0_(1));
    double error_qz_neg = std::abs(q_truth_(5) - (-1)*a0_(2));
    double error_qw_neg = std::abs(q_truth_(6) - (-1)*a0_(3));
    double error_vx = std::abs(v_truth_(0));
    double error_vy = std::abs(v_truth_(1));
    double error_vz = std::abs(v_truth_(2));
    double error_wx = std::abs(w_truth_(0));
    double error_wy = std::abs(w_truth_(1));
    double error_wz = std::abs(w_truth_(2));

    double error_q_sum = error_qx + error_qy + error_qz + error_qw;
    double error_q_sum_neg = error_qx_neg + error_qy_neg + error_qz_neg + error_qw_neg;
    if (error_q_sum_neg < error_q_sum) {
      error_qx = error_qx_neg;
      error_qy = error_qy_neg;
      error_qz = error_qz_neg;
      error_qw = error_qw_neg;
    }

    if (ground_.compare("true") == 0) {
      if (error_x < pos_reg_thresh_ && error_y && pos_reg_thresh_ &&
          error_qx < att_reg_thresh_ && error_qy < att_reg_thresh_ &&
          error_qz < att_reg_thresh_ && error_qw < att_reg_thresh_ &&
          error_vx < vel_reg_thresh_ && error_vy < vel_reg_thresh_ &&
          error_wz < omega_reg_thresh_) {
        target_regulate_finished_ = true;
      }
    } else {
      if (error_x < pos_reg_thresh_ && error_y && pos_reg_thresh_ &&
          error_z < pos_reg_thresh_ && error_qx < att_reg_thresh_ &&
          error_qy < att_reg_thresh_ && error_qz < att_reg_thresh_ &&
          error_qw < att_reg_thresh_ && error_vx < vel_reg_thresh_ &&
          error_vy < vel_reg_thresh_ && error_vz < vel_reg_thresh_ &&
          error_wx < omega_reg_thresh_ && error_wy < omega_reg_thresh_ &&
          error_wz < omega_reg_thresh_) {
        target_regulate_finished_ = true;
      }
    }
  }

  int tumble() {
    /* Tumble function: loop through and tumble. Send out gnc/ctl/command if using default_control.
    */
    int step_cntr = 0;

    // Set publish rate
    float traj_rate = 5.0;
    ros::Rate loop_rate(traj_rate);  // Hz

    if (!default_control_) {  // use CasADi
      publish_casadi_tumble_traj();
      td_control_mode_ = "track";
      td_state_mode_ = "ekf";
    }

    // integrate and publish the trajectory setpoints
    while (step_cntr < input_data_.rows()) {
      if (default_control_) {
        send_tumble_cmd(step_cntr);
      }
      step_cntr++;
      ros::spinOnce();
      loop_rate.sleep();
    }

    NODELET_INFO_STREAM("[TARG COORD]: Tumble finished.");
    test_finished_ = true;
    return 1;
  }

  void get_params() {
    ros::param::getCached("/td/target/x_start", x0_(0));
    ros::param::getCached("/td/target/y_start", x0_(1));
    ros::param::getCached("/td/target/z_start", x0_(2));

    ros::param::getCached("/td/target_coordinator/pos_reg_thresh", pos_reg_thresh_);
    ros::param::getCached("/td/target_coordinator/vel_reg_thresh", vel_reg_thresh_);
    ros::param::getCached("/td/target_coordinator/att_reg_thresh", att_reg_thresh_);
    ros::param::getCached("/td/target_coordinator/omega_reg_thresh", omega_reg_thresh_);
  }

  void disable_default_ctl() {
    /* Switch default controller off now.
    */
    NODELET_DEBUG_STREAM("[TARGET COORDINATOR]: Disabling default controller...");

    // TODO: test speed of this service request

    // Disable the default controller so tube-MPC can run
    std_srvs::SetBool srv;
    srv.request.data = false;
    serv_ctl_enable_.call(srv);

    default_control_ = false;

    NODELET_DEBUG_STREAM("[TARGET COORDINATOR]: Ctl disable service result: " << srv.response.message);
  }

  void enable_default_ctl() {
    /* Switch default controller off now.
    */
    NODELET_DEBUG_STREAM("[TARGET COORDINATOR]: Enabling default controller...");

    // TODO: test speed of this service request

    // Disable the default controller so tube-MPC can run
    std_srvs::SetBool srv;
    srv.request.data = true;
    serv_ctl_enable_.call(srv);

    NODELET_DEBUG_STREAM("[TARGET COORDINATOR]: Ctl enable service result: " << srv.response.message);
    default_control_ = true;
  }

  void testNumberCallback(const trace_msgs::TDTestNumber::ConstPtr& msg) {
    test_number_ = msg->test_number;
    if (test_number_ == -1) {
      // Set flight mode to off
      enable_default_ctl();
      td_flight_mode_ = "off";
      if (!ff_util::FlightUtil::GetFlightMode(flight_mode_, td_flight_mode_)) {
          return;
      }
      pub_flight_mode_.publish(flight_mode_);

      trace_msgs::TDStatus msg;
      msg.stamp = ros::Time::now();
      msg.test_number = test_number_;
      msg.target_coord_ok = target_coord_ok_;
      msg.target_regulate_finished = target_regulate_finished_;
      msg.traj_finished = traj_finished_;
      msg.test_finished = test_finished_;
      msg.default_control = default_control_;
      msg.td_flight_mode = td_flight_mode_;
      msg.td_control_mode = td_control_mode_;
      msg.td_state_mode = td_state_mode_;
      msg.casadi_on_target = casadi_on_target_;
      pub_status_.publish(msg);
    }
  }

  void ekfCallback(const ff_msgs::EkfState::ConstPtr& msg) {
          /*
          The `gnc/ekf` subscriber callback
          Called at 62.5 Hz
          */
          // NODELET_INFO_STREAM("EKF callback called");

          float qx = msg->pose.orientation.x;
          float qy = msg->pose.orientation.y;
          float qz = msg->pose.orientation.z;
          float qw = msg->pose.orientation.w;

          if (qx != 0 || qy != 0 || qz != 0 || qw != 0) {
            p_truth_(0) = msg->pose.position.x;
            p_truth_(1) = msg->pose.position.y;
            p_truth_(2) = msg->pose.position.z;
            q_truth_(0) = msg->pose.orientation.x;
            q_truth_(1) = msg->pose.orientation.y;
            q_truth_(2) = msg->pose.orientation.z;
            q_truth_(3) = msg->pose.orientation.w;
            v_truth_(0) = msg->velocity.x;
            v_truth_(1) = msg->velocity.y;
            v_truth_(2) = msg->velocity.z;
            w_truth_(0) = msg->omega.x;
            w_truth_(1) = msg->omega.y;
            w_truth_(2) = msg->omega.z;
          }
  }

  void send_default_regulate_cmd() {
    geometry_msgs::Point reg_pos;
    geometry_msgs::Vector3 reg_vel;
    geometry_msgs::Quaternion reg_quat;
    geometry_msgs::Vector3 reg_omega;

    reg_pos.x = x0_(0);
    reg_pos.y = x0_(1);
    reg_pos.z = x0_(2);

    // velocity
    reg_vel.x = 0.0;
    reg_vel.y = 0.0;
    reg_vel.z = 0.0;

    // attitude
    reg_quat.w = a0_(3);
    reg_quat.x = a0_(0);
    reg_quat.y = a0_(1);
    reg_quat.z = a0_(2);

    // angular rate
    reg_omega.x = 0.0;
    reg_omega.y = 0.0;
    reg_omega.z = 0.0;

    // desired acceleration is 0
    geometry_msgs::Vector3 reg_accel;
    reg_accel.x = 0.0;
    reg_accel.y = 0.0;
    reg_accel.z = 0.0;

    // desired angular acceleration is 0
    geometry_msgs::Vector3 reg_alpha;
    reg_alpha.x = 0.0;
    reg_alpha.y = 0.0;
    reg_alpha.z = 0.0;

    // Publish the tumbling traj topic
    static ff_msgs::ControlState reg;
    reg.when = ros::Time::now();;
    reg.pose.position = reg_pos;
    reg.pose.orientation = reg_quat;
    reg.twist.linear = reg_vel;
    reg.twist.angular = reg_omega;
    reg.accel.linear = reg_accel;
    reg.accel.angular = reg_alpha;

    pub_gnc_ctl_setpoint_.publish(reg);
  }

  void publish_casadi_tumble_traj() {
    eigen_x_des_traj_.resize(input_data_.rows(), 14);
    for (int i = 0; i < input_data_.rows(); i++) {
      eigen_x_des_traj_(i, 0) = input_data_(i, 0);
      eigen_x_des_traj_(i, 1) = x0_(0);
      eigen_x_des_traj_(i, 2) = x0_(1);
      eigen_x_des_traj_(i, 3) = x0_(2);
      eigen_x_des_traj_(i, 4) = 0.0;
      eigen_x_des_traj_(i, 5) = 0.0;
      eigen_x_des_traj_(i, 6) = 0.0;

      eigen_x_des_traj_(i, 7) = input_data_(i, 8);
      eigen_x_des_traj_(i, 8) = input_data_(i, 9);
      eigen_x_des_traj_(i, 9) = input_data_(i, 10);
      eigen_x_des_traj_(i, 10) = input_data_(i, 7);

      eigen_x_des_traj_(i, 11) = input_data_(i, 11);
      eigen_x_des_traj_(i, 12) = input_data_(i, 12);
      eigen_x_des_traj_(i, 13) = input_data_(i, 13);
    }

    std_msgs::Float64MultiArray eigen_x_des_traj_msg;
    tf::matrixEigenToMsg(eigen_x_des_traj_, eigen_x_des_traj_msg);

    pub_x_des_traj_.publish(eigen_x_des_traj_msg);
  }

  void PublishTDStatus(const ros::TimerEvent& t) {
    trace_msgs::TDStatus msg;
    msg.stamp = ros::Time::now();
    msg.test_number = test_number_;
    msg.target_coord_ok = target_coord_ok_;
    msg.target_regulate_finished = target_regulate_finished_;
    msg.traj_finished = traj_finished_;
    msg.test_finished = test_finished_;
    msg.default_control = default_control_;
    msg.td_flight_mode = td_flight_mode_;
    msg.td_control_mode = td_control_mode_;
    msg.td_state_mode = td_state_mode_;
    msg.casadi_on_target = casadi_on_target_;
    msg.gain_mode = td_gain_mode_;
    pub_status_.publish(msg);
  }

  void send_tumble_cmd(const int index) {
    /* Send a gnc/ctl/command
    */
    // extract the current value from the desired trajectory array (0 is the time, skip it)
    float qw_traj = input_data_(index, 7);
    float qx_traj = input_data_(index, 8);
    float qy_traj = input_data_(index, 9);
    float qz_traj = input_data_(index, 10);
    float wx_traj = input_data_(index, 11);
    float wy_traj = input_data_(index, 12);
    float wz_traj = input_data_(index, 13);

    // set up the trajectory publishing messages
    geometry_msgs::Point traj_pos;
    geometry_msgs::Vector3 traj_vel;
    geometry_msgs::Quaternion traj_quat;
    geometry_msgs::Vector3 traj_omega;

    // position (corresponding to initial ISS position)
    traj_pos.x = x0_(0);
    traj_pos.y = x0_(1);
    traj_pos.z = x0_(2);

    // velocity
    traj_vel.x = 0.0;
    traj_vel.y = 0.0;
    traj_vel.z = 0.0;

    // attitude
    traj_quat.w = qw_traj;
    traj_quat.x = qx_traj;
    traj_quat.y = qy_traj;
    traj_quat.z = qz_traj;
    Vector4f q; q << qx_traj, qy_traj, qz_traj, qw_traj;
    if (q.norm() < 0.999 || q.norm() > 1.001) {
      traj_quat.w = 1.0;
      traj_quat.x = 0.0;
      traj_quat.y = 0.0;
      traj_quat.z = 0.0;
      target_coord_ok_ = false;
    }

    // angular rate
    traj_omega.x = wx_traj;
    traj_omega.y = wy_traj;
    traj_omega.z = wz_traj;
    if (wx_traj > max_omega_) {
      traj_omega.x = max_omega_;
      target_coord_ok_ = false;
    }
    if (wy_traj > max_omega_) {
      traj_omega.y = max_omega_;
      target_coord_ok_ = false;
    }
    if (wz_traj > max_omega_) {
      traj_omega.z = max_omega_;
      target_coord_ok_ = false;
    }

    // desired acceleration is 0
    geometry_msgs::Vector3 traj_accel;
    traj_accel.x = 0.0;
    traj_accel.y = 0.0;
    traj_accel.z = 0.0;

    // desired angular acceleration is 0
    geometry_msgs::Vector3 traj_alpha;
    traj_alpha.x = 0.0;
    traj_alpha.y = 0.0;
    traj_alpha.z = 0.0;

    // Publish the tumbling traj topic
    static ff_msgs::ControlState tumble;
    tumble.when = ros::Time::now();;
    tumble.pose.position = traj_pos;
    tumble.pose.orientation = traj_quat;
    tumble.twist.linear = traj_vel;
    tumble.twist.angular = traj_omega;
    tumble.accel.linear = traj_accel;
    tumble.accel.angular = traj_alpha;

    pub_gnc_ctl_setpoint_.publish(tumble);
  }
};
}  // end namespace target_coordinator

// Declare the plugin
PLUGINLIB_EXPORT_CLASS(target_coordinator::TargetCoordinatorNodelet, nodelet::Nodelet);
