/**
 * @file uc_bound_nodelet.cc
 * @author Charles Oestreich
 * @brief Nodelet to calculate uncertainty bounds in the chaser satellite's inertial trajectory.
 * @version 0.1
 * @date 2023-04-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#define _USE_MATH_DEFINES

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <boost/numeric/odeint/algebra/vector_space_algebra.hpp>
#include <boost/numeric/odeint/integrate/integrate.hpp>
#include <boost/numeric/odeint/stepper/runge_kutta_dopri5.hpp>
#include <cmath>
#include <fstream>
#include <random>
#include <sstream>
#include <thread>
#include <vector>

// ROS includes
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/package.h>

// FSW includes
#include <ff_msgs/EkfState.h>
#include <ff_util/ff_names.h>
#include <ff_util/ff_nodelet.h>
#include <ff_msg_conversions/ff_msg_conversions.h>

// Custom includes
#include <string.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/Inertia.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>

// Utilities
#include <trace_msgs/TDStatus.h>
#include <trace_msgs/TDUCBoundStatus.h>
#include "data/eigen_kdl.h"
#include "data/eigen_msg.h"
#include "data/att_utils_f.h"  // float version
#include "data/csv_read.h"

float EPSILON = std::numeric_limits<float>::epsilon();

using namespace Eigen;
using namespace boost::numeric::odeint;

// Set type for state vector
typedef Matrix<float, 1, 7> state_type;

// Using quaternion convention: q = [qx, qy, qz, qw]

namespace uc_bound {

class UCBoundNodelet : public ff_util::FreeFlyerNodelet {
 public:
  UCBoundNodelet() : ff_util::FreeFlyerNodelet(true) {}
  ~UCBoundNodelet() {}

 private:
  // Parameters
  std::string instruction_ = "no_action";  // TRACE instruction parameter, set by ASAP
  std::string sim_ = "false";
  std::string traj_filename_;
  int test_number_ = 0;
  int num_trials_;
  float max_angle_error_;
  float max_w_error_;
  float init_max_angle_error_;
  float init_max_w_error_;
  float max_J_axis_1_;
  float max_J_axis_2_;
  float max_J_axis_3_;
  float max_J_prod_1_;
  float max_J_prod_2_;
  float max_J_prod_3_;
  float traj_dt_;
  float traj_rate_;
  int N_traj_;
  bool have_traj_ = false;

  bool unit_test_complete_ = false;

  // 0 = pre-saved CSV trajectory for unit test
  // 1 = standard online motion planner format
  int traj_type_;

  float loop_rate_ = 1;

  bool activate_ = false;
  bool uc_bound_finished_ = false;

  Vector3f targ_offset_;

  MatrixXf eigen_x_des_traj_;

  // Publisher for UC bound
  ros::Publisher pub_uc_bound_;
  ros::Publisher pub_uc_bound_status_;

  ros::Subscriber sub_status_;

  // Subscribers for initial pose estimate
  ros::Subscriber sub_targ_pose0_;
  ros::Subscriber sub_targ_twist0_;
  Vector4f targ_att0_;
  Vector3f targ_w0_;

  // Subscriber for motion plan
  ros::Subscriber sub_x_des_traj_;
  ros::Subscriber sub_inertia_;

  std::string ground_ = "false";

  Matrix3f J_;
  std::vector<float> J_nom_vec_;

  int num_unit_iters_;

  std::shared_ptr<std::thread> thread_;

  // dynamic propagator stuff
  class dynamic_step_ {
      Matrix3f J_param;

     public:
      dynamic_step_(Matrix3f G) : J_param(G) {}

      void operator()(state_type const& x, state_type& dx,
                      const float& t) const {
        VectorXf q = x.head(4);
        Vector3f w = x.segment(4, 3);
        Vector3f M =
            VectorXf::Zero(3);  // Assuming no external torques on target
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

      void operator()(state_type const& x, state_type& dx,
                      const float& t) const {
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

  void Initialize(ros::NodeHandle* nh) {
    /* This is called when the nodelet is loaded into the nodelet manager
    */
    ros::param::getCached("/td/sim", sim_);

    // subscribers
    sub_targ_pose0_ = nh->subscribe<geometry_msgs::PoseWithCovariance>(
        "/td/motion_planner_interface/target_traj_pose0", 5, &UCBoundNodelet::poseCallback,
        this);
    sub_targ_twist0_ = nh->subscribe<geometry_msgs::TwistWithCovariance>(
        "/td/motion_planner_interface/target_traj_twist0", 5,
        &UCBoundNodelet::twistCallback, this);
    sub_x_des_traj_ = nh->subscribe<std_msgs::Float64MultiArray>(
        "td/tube_mpc/traj", 5, &UCBoundNodelet::trajCallback, this);
    sub_status_ = nh->subscribe<trace_msgs::TDStatus>("td/status", 5, &UCBoundNodelet::statusCallback, this);
    sub_inertia_ = nh->subscribe<geometry_msgs::Inertia>(
        "/td/mit_slam/inertia", 5, &UCBoundNodelet::inertiaCallback, this);

    // publishers
    pub_uc_bound_ = nh->advertise<std_msgs::Float64MultiArray>("/td/uc_bound/uc_bound", 5, true);
    pub_uc_bound_status_ = nh->advertise<trace_msgs::TDUCBoundStatus>("/td/uc_bound/status", 5, true);

    NODELET_INFO_STREAM("[UC BOUND] Initialized.");

    thread_.reset(new std::thread(&uc_bound::UCBoundNodelet::Run, this));
  }

  void Run() {
    /* Main ROS loop.
    */
    ros::Rate spin_rate(2.0);
    while (ros::ok()) {
      int result = full_test();
      ros::spinOnce();
      spin_rate.sleep();
    }
  }

  int full_test() {
    /* Check if we're ready to perform a uc_bound analysis.
    */
    ros::param::getCached("/td/ground", ground_);

    std::vector<double> targ_offset_ros;
    if (ground_.compare("true") == 0) {
      ros::param::getCached("/td/uc_bound/targ_offset_ground", targ_offset_ros);
    } else {
      ros::param::getCached("/td/uc_bound/targ_offset_iss", targ_offset_ros);
    }
    // ros::param::getCached("/td/uc_bound/targ_offset", targ_offset_ros);
    targ_offset_ << targ_offset_ros[0], targ_offset_ros[1], targ_offset_ros[2];

    // Nominal target inertia
    std::vector<double> J_vec_target;
    ros::param::getCached("/td/uc_bound/target_J", J_vec_target);
    int k = 0;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        J_(i, j) = J_vec_target[k];
        k++;
      }
    }

    traj_type_ = 1;

    ros::Rate spin_rate(2.0);
    ros::param::getCached("/td/uc_bound/traj_dt", traj_dt_);
    while (ros::ok()) {
      if (activate_ && !uc_bound_finished_ && have_traj_) {
        // Get motion planner file and dt
        MatrixXf nom_chaser_traj = eigen_x_des_traj_;

        // Calculate uncertainty bound
        ros::param::getCached("/td/uc_bound/mc_trials", num_trials_);
        ros::param::getCached("/td/uc_bound/init_max_angle_error", init_max_angle_error_);
        ros::param::getCached("/td/uc_bound/init_max_w_error", init_max_w_error_);
        ros::param::getCached("/td/uc_bound/max_angle_error", max_angle_error_);
        ros::param::getCached("/td/uc_bound/max_w_error", max_w_error_);
        ros::param::getCached("/td/uc_bound/max_J_axis_1", max_J_axis_1_);
        ros::param::getCached("/td/uc_bound/max_J_axis_2", max_J_axis_2_);
        ros::param::getCached("/td/uc_bound/max_J_axis_3", max_J_axis_3_);
        ros::param::getCached("/td/uc_bound/max_J_prod_1", max_J_prod_1_);
        ros::param::getCached("/td/uc_bound/max_J_prod_2", max_J_prod_2_);
        ros::param::getCached("/td/uc_bound/max_J_prod_3", max_J_prod_3_);
        MatrixXf eig_uc_bound_mat;

        eig_uc_bound_mat = calc_uc_bound(nom_chaser_traj, targ_att0_, targ_w0_, num_trials_);

        std_msgs::Float64MultiArray uc_bound_mat;
        tf::matrixEigenToMsg(eig_uc_bound_mat, uc_bound_mat);
        pub_uc_bound_.publish(uc_bound_mat);

        std::cout << "[UC BOUND]: Calculated uncertainty bound: \n" << eig_uc_bound_mat << std::endl;


        uc_bound_finished_ = true;
        trace_msgs::TDUCBoundStatus msg;
        msg.stamp = ros::Time::now();
        msg.uc_bound_finished = uc_bound_finished_;
        pub_uc_bound_status_.publish(msg);
      }
      ros::spinOnce();
      spin_rate.sleep();
    }
    return 1;
  }

  void statusCallback(const trace_msgs::TDStatus::ConstPtr& msg) {
    activate_ = msg->uc_bound_activate;
    test_number_ = msg->test_number;
  }

  /*
  Target tumble estimate callback functions
  */
  void poseCallback(const geometry_msgs::PoseWithCovariance::ConstPtr& pose_msg) {
    targ_att0_(0) = pose_msg->pose.orientation.x;
    targ_att0_(1) = pose_msg->pose.orientation.y;
    targ_att0_(2) = pose_msg->pose.orientation.z;
    targ_att0_(3) = pose_msg->pose.orientation.w;
  }

  void twistCallback(const geometry_msgs::TwistWithCovariance::ConstPtr& twist_msg) {
    targ_w0_(0) = twist_msg->twist.angular.x;
    targ_w0_(1) = twist_msg->twist.angular.y;
    targ_w0_(2) = twist_msg->twist.angular.z;
  }

  void trajCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    int i = msg->layout.dim[0].size;
    int j = msg->layout.dim[1].size;
    std::vector<double> data = msg->data;
    eigen_x_des_traj_ = Eigen::Map<Eigen::MatrixXd>(data.data(), j, i).transpose().cast<float>();

    traj_rate_ = 1.0/traj_dt_;
    have_traj_ = true;
    // std::cout << "[UC BOUND]: traj: " << std::endl <<  eigen_x_des_traj_ <<
    // std::endl;
  }

  void inertiaCallback(const geometry_msgs::Inertia::ConstPtr& inertia_msg) {
    J_(0, 0) = inertia_msg->ixx;
    J_(0, 1) = inertia_msg->ixy;
    J_(0, 2) = inertia_msg->ixz;
    J_(1, 1) = inertia_msg->iyy;
    J_(1, 2) = inertia_msg->iyz;
    J_(2, 2) = inertia_msg->izz;
    J_(1, 0) = J_(0, 1);
    J_(2, 1) = J_(1, 2);
    J_(2, 0) = J_(0, 2);
  }

  /*
  Calculate uncertainty bound
  */
  MatrixXf calc_uc_bound(const MatrixXf& nom_chaser_traj_data,
                         const Vector4f& est_targ_att0,
                         const Vector3f& est_targ_w0, const int& num_trials) {
    /*******************
    1. Organize nominal chaser trajectory data
    *******************/
    int num_rows = nom_chaser_traj_data.rows();
    MatrixXf nom_chaser_pos(num_rows, 3);
    MatrixXf nom_chaser_vel(num_rows, 3);

    // Center pre-saved trajectory on target
    if (traj_type_ == 0) {
      for (int i = 0; i < 3; i++) {
        nom_chaser_pos.col(i) = nom_chaser_traj_data.col(i + 1);
        if (i == 1) {
          nom_chaser_pos.col(i) =
              nom_chaser_traj_data.col(i + 1) -
              3.0 * VectorXf::Ones(
                        num_rows);  // Make it target-centered in y-axis
        }
        nom_chaser_vel.col(i) = nom_chaser_traj_data.col(i + 4);
      }
    }
    // Center online motion planner on target
    // Time is gone from online motion planner trajectory read in, thus use
    // col(i) not col(i+1) Basing pos, vel, acc columns off of traj_utils.h
    // (get_traj_output_x)
    else {
      for (int i = 0; i < 3; i++) {
        nom_chaser_pos.col(i) = nom_chaser_traj_data.col(1 + i) -
                                targ_offset_(i) * VectorXf::Ones(num_rows);
        nom_chaser_vel.col(i) = nom_chaser_traj_data.col(4 + i);
      }
    }

    // Take away z-axis of position
    if (ground_.compare("true") == 0) {
        nom_chaser_pos.col(2) = VectorXf::Zero(num_rows);
        nom_chaser_vel.col(2) = VectorXf::Zero(num_rows);
    }

    /*******************
     2. Get reference trajectory in body frame
    *******************/
    Vector4f nom_targ_q0 = est_targ_att0;
    Matrix3f nom_targ_R0 = q2dcm(nom_targ_q0);
    Vector3f nom_omega_targ0_I = nom_targ_R0 * est_targ_w0;

    MatrixXf nom_targ_traj(num_rows, 7);
    nom_targ_traj(0, 0) = nom_targ_q0(0);
    nom_targ_traj(0, 1) = nom_targ_q0(1);
    nom_targ_traj(0, 2) = nom_targ_q0(2);
    nom_targ_traj(0, 3) = nom_targ_q0(3);
    nom_targ_traj(0, 4) = est_targ_w0(0);
    nom_targ_traj(0, 5) = est_targ_w0(1);
    nom_targ_traj(0, 6) = est_targ_w0(2);

    MatrixXf nom_chaser_pos_targ(num_rows, 3);
    nom_chaser_pos_targ.row(0) = nom_targ_R0 * nom_chaser_pos.row(0).transpose();
    MatrixXf nom_chaser_vel_targ(num_rows, 3);
    nom_chaser_vel_targ.row(0) =
        nom_targ_R0 * nom_chaser_vel.row(0).transpose() +
        nom_omega_targ0_I.cross(nom_targ_R0 *
                                nom_chaser_pos.row(0).transpose());

    state_type x;
    x = nom_targ_traj.row(0);

    runge_kutta_dopri5<state_type, double, state_type, double, vector_space_algebra> stepper;

    // **** Boost RK4 method ****
    double t = 0.0;
    for (int i = 1; i < num_rows; i++) {
      double t_next = t + traj_dt_;
      if (ground_.compare("true") == 0) {
        integrate_adaptive(stepper, dynamic_step_ground_{J_}, x, t, t_next,
                           (double)traj_dt_);
      } else {
        integrate_adaptive(stepper, dynamic_step_{J_}, x, t, t_next,
                           (double)traj_dt_);
      }
      t += traj_dt_;

      nom_targ_traj.row(i) = x;
      Matrix3f nom_targ_R = q2dcm(nom_targ_traj.row(i).head(4));
      Vector3f nom_omega_targ_I = nom_targ_R * x.tail(3).transpose();

      // Get the nominal chaser trajectory in the target frame
      nom_chaser_pos_targ.row(i) = nom_targ_R * nom_chaser_pos.row(i).transpose();
      nom_chaser_vel_targ.row(i) =
          nom_targ_R * nom_chaser_vel.row(i).transpose() +
          nom_omega_targ_I.cross(nom_targ_R *
                                 nom_chaser_pos.row(i).transpose());
    }

    /*******************
    3. Monte Carlo trials with perturbed initial target attitude/angular velocity
    *******************/
    // Define attitude/w perturbation bounds
    Vector3f est_targ_att0_vec;
    est_targ_att0_vec = q2vec(est_targ_att0);

    int success_count = 0;

    Vector3f uc_bound_pos;
    Vector3f uc_bound_vel;
    MatrixXf uncertainty_bound(4, 3);
    MatrixXf w_pos_data(num_trials*num_rows, 3);
    MatrixXf w_vel_data(num_trials*num_rows, 3);

    // Motion constraint parameters:
    std::vector<float> p_constraint_ros;
    if (ground_.compare("true") == 0) {
        ros::param::getCached("/td/uc_bound/p_constraint_ground", p_constraint_ros);
    } else {
      ros::param::getCached("/td/uc_bound/p_constraint_iss", p_constraint_ros);
    }
    Vector3f p_constraint;
    p_constraint << p_constraint_ros[0], p_constraint_ros[1], p_constraint_ros[2];  // meters

    std::vector<float> p_constraint_center_ros;
    if (ground_.compare("true") == 0) {
        ros::param::getCached("/td/uc_bound/p_constraint_center_ground", p_constraint_center_ros);
    } else {
      ros::param::getCached("/td/uc_bound/p_constraint_center_iss",
                            p_constraint_center_ros);
    }
    Vector3f p_constraint_center;
    p_constraint_center << p_constraint_center_ros[0],
        p_constraint_center_ros[1], p_constraint_center_ros[2];  // meters

    std::vector<float> v_constraint_ros;
    if (ground_.compare("true") == 0) {
        ros::param::getCached("/td/uc_bound/v_constraint_ground", v_constraint_ros);
    } else {
      ros::param::getCached("/td/uc_bound/v_constraint_iss", v_constraint_ros);
    }
    Vector3f v_constraint;
    v_constraint << v_constraint_ros[0], v_constraint_ros[1], v_constraint_ros[2];  // m/s

    // parameters for random perturbation to initial target state:
    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::uniform_real_distribution<float> att01_dis(-init_max_angle_error_, init_max_angle_error_);
    std::uniform_real_distribution<float> att02_dis(-init_max_angle_error_, init_max_angle_error_);
    std::uniform_real_distribution<float> att03_dis(-init_max_angle_error_, init_max_angle_error_);
    std::uniform_real_distribution<float> w0x_dis(-init_max_w_error_, init_max_w_error_);
    std::uniform_real_distribution<float> w0y_dis(-init_max_w_error_, init_max_w_error_);
    std::uniform_real_distribution<float> w0z_dis(-init_max_w_error_, init_max_w_error_);

    std::uniform_real_distribution<float> att1_dis(-max_angle_error_, max_angle_error_);
    std::uniform_real_distribution<float> att2_dis(-max_angle_error_, max_angle_error_);
    std::uniform_real_distribution<float> att3_dis(-max_angle_error_, max_angle_error_);
    std::uniform_real_distribution<float> wx_dis(-max_w_error_, max_w_error_);
    std::uniform_real_distribution<float> wy_dis(-max_w_error_, max_w_error_);
    std::uniform_real_distribution<float> wz_dis(-max_w_error_, max_w_error_);

    std::uniform_real_distribution<float> J_axis_dis_1(-max_J_axis_1_, max_J_axis_1_);
    std::uniform_real_distribution<float> J_axis_dis_2(-max_J_axis_2_, max_J_axis_2_);
    std::uniform_real_distribution<float> J_axis_dis_3(-max_J_axis_3_, max_J_axis_3_);
    std::uniform_real_distribution<float> J_prod_dis_1(-max_J_prod_1_, max_J_prod_1_);
    std::uniform_real_distribution<float> J_prod_dis_2(-max_J_prod_2_, max_J_prod_2_);
    std::uniform_real_distribution<float> J_prod_dis_3(-max_J_prod_3_, max_J_prod_3_);

    for (int k = 0; k < num_trials_; k++) {
      // randomly initialize attitude/angular velocity
      Vector4f real_targ_att0;
      real_targ_att0 << 0.0, 0.0, 0.0, 1.0;
      Vector3f real_targ_w0;
      real_targ_w0 << 0.0, 0.0, 0.0;
      Vector3f att0_vec_perturb;
      att0_vec_perturb(0) = att01_dis(gen);
      att0_vec_perturb(1) = att02_dis(gen);
      att0_vec_perturb(2) = att03_dis(gen);
      Vector3f w0_perturb;
      w0_perturb(0) = w0x_dis(gen);
      w0_perturb(1) = w0y_dis(gen);
      w0_perturb(2) = w0z_dis(gen);
      if (ground_.compare("true") == 0) {
        att0_vec_perturb(0) = 0.0;
        att0_vec_perturb(1) = 0.0;
        w0_perturb(0) = 0.0;
        w0_perturb(1) = 0.0;
      }


      real_targ_att0 = vec2q(est_targ_att0_vec + att0_vec_perturb);
      real_targ_w0 = est_targ_w0 + w0_perturb;

      // randomly perturb target inertia
      bool inertia_check = false;
      Eigen::Matrix3f real_J;
      while (inertia_check == 0) {
        real_J(0, 0) = J_(0, 0) + J_axis_dis_1(gen);
        real_J(1, 1) = J_(0, 0) + J_axis_dis_2(gen);
        real_J(2, 2) = J_(0, 0) + J_axis_dis_3(gen);

        real_J(0, 1) = J_(0, 1) + J_prod_dis_1(gen);
        real_J(1, 0) = real_J(0, 1);
        real_J(0, 2) = J_(0, 2) + J_prod_dis_2(gen);
        real_J(2, 0) = real_J(0, 2);
        real_J(1, 2) = J_(1, 2) + J_prod_dis_3(gen);
        real_J(2, 1) = real_J(1, 2);

        if ((real_J(0, 0) + real_J(1, 1) >= real_J(2, 2)) &&
            (real_J(0, 0) + real_J(2, 2) >= real_J(1, 1)) &&
            (real_J(1, 1) + real_J(2, 2) >= real_J(0, 0))) {
          inertia_check = true;
        }
      }

      // Now propagate the perturbed target motion
      Vector4f real_targ_q0 = real_targ_att0;
      Matrix3f real_targ_R0 = q2dcm(real_targ_q0);

      MatrixXf real_targ_traj(num_rows, 7);
      real_targ_traj(0, 0) = real_targ_q0(0);
      real_targ_traj(0, 1) = real_targ_q0(1);
      real_targ_traj(0, 2) = real_targ_q0(2);
      real_targ_traj(0, 3) = real_targ_q0(3);
      real_targ_traj(0, 4) = real_targ_w0(0);
      real_targ_traj(0, 5) = real_targ_w0(1);
      real_targ_traj(0, 6) = real_targ_w0(2);

      state_type x;
      x = real_targ_traj.row(0);



      t = 0.0;
      for (int i = 1; i < num_rows; i++) {
        double t_next = t + traj_dt_;
        if (ground_.compare("true") == 0) {
          integrate_adaptive(stepper, dynamic_step_ground_{real_J}, x, t,
                             t_next, (double)traj_dt_);
        } else {
          integrate_adaptive(stepper, dynamic_step_{real_J}, x, t, t_next,
                             (double)traj_dt_);
        }
        t += traj_dt_;

        real_targ_traj.row(i) = x;
        Matrix3f real_targ_R = q2dcm(real_targ_traj.row(i).head(4));
      }

      // Compute step-by-step uncertainty
      MatrixXf pos_uncertainty_trial(num_rows, 3);
      MatrixXf vel_uncertainty_trial(num_rows, 3);
      pos_uncertainty_trial.row(0) = Eigen::Vector3f::Zero();
      vel_uncertainty_trial.row(0) = Eigen::Vector3f::Zero();
      Vector3f x_pos;
      Vector3f x_vel;
      Vector3f z_pos;
      Vector3f z_vel;
      MatrixXf real_chaser_pos(num_rows, 3);
      real_chaser_pos.row(0) = nom_chaser_pos.row(0);
      MatrixXf real_chaser_vel(num_rows, 3);
      real_chaser_vel.row(0) = nom_chaser_vel.row(0);

      MatrixXf est_targ_traj(num_rows, 7);
      est_targ_traj(0, 0) = est_targ_att0(0);
      est_targ_traj(0, 1) = est_targ_att0(1);
      est_targ_traj(0, 2) = est_targ_att0(2);
      est_targ_traj(0, 3) = est_targ_att0(3);
      est_targ_traj(0, 4) = est_targ_w0(0);
      est_targ_traj(0, 5) = est_targ_w0(1);
      est_targ_traj(0, 6) = est_targ_w0(2);

      z_pos = nom_chaser_pos.row(1);
      z_vel = nom_chaser_vel.row(1);

      t = 0.0;
      for (int i = 1; i < num_rows; i++) {
        // Updated state estimate
        Vector3f vec_real = q2vec(real_targ_traj.row(i).head(4));
        Vector3f vec_est;
        vec_est << 0.0, 0.0, 0.0;
        Vector3f vec_perturb;
        vec_perturb(0) = att1_dis(gen);
        vec_perturb(1) = att2_dis(gen);
        vec_perturb(2) = att3_dis(gen);
        if (ground_.compare("true") == 0) {
          vec_perturb(0) = 0.0;
          vec_perturb(1) = 0.0;
        }
        vec_est = vec_real + vec_perturb;
        Vector4f q_est = vec2q(vec_est);
        Matrix3f R_est = q2dcm(q_est);

        Vector3f w_real = real_targ_traj.row(i).tail(3);
        Vector3f w_est;
        w_est << 0.0, 0.0, 0.0;
        Vector3f w_perturb;
        w_perturb(0) = wx_dis(gen);
        w_perturb(1) = wy_dis(gen);
        w_perturb(2) = wz_dis(gen);
        if (ground_.compare("true") == 0) {
          w_perturb(0) = 0.0;
          w_perturb(1) = 0.0;
        }
        w_est = w_real + w_perturb;
        Vector3f w_est_I = R_est * w_est;

        // Compute disturbance
        x_pos = R_est.transpose() * nom_chaser_pos_targ.row(i).transpose();
        x_vel = R_est.transpose() * (nom_chaser_vel_targ.row(i).transpose() - w_est_I.cross(R_est * x_pos));

        real_chaser_pos.row(i) = x_pos;
        real_chaser_vel.row(i) = x_vel;

        pos_uncertainty_trial.row(i) = x_pos - z_pos;
        vel_uncertainty_trial.row(i) = x_vel - z_vel;

        // Propagate state estimate to get a prediction of next step's attitude
        if (i != num_rows) {
          state_type x_est;
          x_est(0) = q_est(0);
          x_est(1) = q_est(1);
          x_est(2) = q_est(2);
          x_est(3) = q_est(3);
          x_est(4) = w_est(0);
          x_est(5) = w_est(1);
          x_est(6) = w_est(2);

          double t_next = t + traj_dt_;
          if (ground_.compare("true") == 0) {
            integrate_adaptive(stepper, dynamic_step_ground_{J_}, x_est, t,
                               t_next, (double)traj_dt_);
          } else {
            integrate_adaptive(stepper, dynamic_step_{J_}, x_est, t, t_next,
                               (double)traj_dt_);
          }
          t += traj_dt_;

          Vector4f q_prop = x_est.head(4);
          Vector3f w_prop = x_est.tail(3).transpose();
          Matrix3f R_prop = q2dcm(q_prop);
          Vector3f w_prop_I = R_prop * w_prop;

          z_pos = R_prop.transpose() * nom_chaser_pos_targ.row(i+1).transpose();
          z_vel = R_prop.transpose() * (nom_chaser_vel_targ.row(i+1).transpose() - w_prop_I.cross(R_prop * z_pos));
        }
      }

      // Analyze the resulting chaser trajectory to determine if motion constraints have been violated
      float max_x = real_chaser_pos.col(0).maxCoeff();
      float min_x = real_chaser_pos.col(0).minCoeff();
      float max_y = real_chaser_pos.col(1).maxCoeff();
      float min_y = real_chaser_pos.col(1).minCoeff();
      float max_z = real_chaser_pos.col(2).maxCoeff();
      float min_z = real_chaser_pos.col(2).minCoeff();

      MatrixXf abs_vel = real_chaser_vel.cwiseAbs();
      float max_vx = abs_vel.col(0).maxCoeff();
      float max_vy = abs_vel.col(1).maxCoeff();
      float max_vz = abs_vel.col(2).maxCoeff();

      int success_pos = 0;
      int success_vel = 0;

      // Check to see if trajectory violated constraints
      if (max_x < (p_constraint_center(0) + p_constraint(0)) &&
          max_y < (p_constraint_center(1) + p_constraint(1)) &&
          max_z < (p_constraint_center(2) + p_constraint(2)) &&
          min_x > (p_constraint_center(0) - p_constraint(0)) &&
          min_y > (p_constraint_center(1) - p_constraint(1)) &&
          min_z > (p_constraint_center(2) - p_constraint(2))) {
        success_pos = 1;
      }

      if (max_vx < v_constraint(0) && max_vy < v_constraint(1) &&
          max_vz < v_constraint(2)) {
        success_vel = 1;
      }
      // only add to w data if the trajectory did not violate constraints
      if (success_pos == 1 && success_vel == 1) {
        success_count++;
        int idx = 0;
        for (int n = k * num_rows; n <= (k + 1) * num_rows - 1; n++) {
          w_pos_data(n, 0) = pos_uncertainty_trial(idx, 0);
          w_pos_data(n, 1) = pos_uncertainty_trial(idx, 1);
          w_pos_data(n, 2) = pos_uncertainty_trial(idx, 2);
          w_vel_data(n, 0) = vel_uncertainty_trial(idx, 0);
          w_vel_data(n, 1) = vel_uncertainty_trial(idx, 1);
          w_vel_data(n, 2) = vel_uncertainty_trial(idx, 2);
          idx++;
        }

      } else {
        int idx = 0;
        for (int n = k * num_rows; n <= (k + 1) * num_rows - 1; n++) {
          w_pos_data(n, 0) = 0.0;
          w_pos_data(n, 1) = 0.0;
          w_pos_data(n, 2) = 0.0;
          w_vel_data(n, 0) = 0.0;
          w_vel_data(n, 1) = 0.0;
          w_vel_data(n, 2) = 0.0;
          idx++;
        }
      }

    }  // End Monte Carlo trials

    MatrixXf abs_w_pos_data = w_pos_data.cwiseAbs();
    float max_w_pos_x = abs_w_pos_data.col(0).maxCoeff();
    float max_w_pos_y = abs_w_pos_data.col(1).maxCoeff();
    float max_w_pos_z = abs_w_pos_data.col(2).maxCoeff();

    MatrixXf abs_w_vel_data = w_vel_data.cwiseAbs();
    float max_w_vel_x = abs_w_vel_data.col(0).maxCoeff();
    float max_w_vel_y = abs_w_vel_data.col(1).maxCoeff();
    float max_w_vel_z = abs_w_vel_data.col(2).maxCoeff();

    // Return the maximum position/velocity deviations for successful trials as the uncertainty bound
    uncertainty_bound(0, 0) = max_w_pos_x;
    uncertainty_bound(0, 1) = max_w_pos_y;
    uncertainty_bound(0, 2) = max_w_pos_z;
    uncertainty_bound(1, 0) = -max_w_pos_x;
    uncertainty_bound(1, 1) = -max_w_pos_y;
    uncertainty_bound(1, 2) = -max_w_pos_z;
    uncertainty_bound(2, 0) = max_w_vel_x;
    uncertainty_bound(2, 1) = max_w_vel_y;
    uncertainty_bound(2, 2) = max_w_vel_z;
    uncertainty_bound(3, 0) = -max_w_vel_x;
    uncertainty_bound(3, 1) = -max_w_vel_y;
    uncertainty_bound(3, 2) = -max_w_vel_z;

    if (ground_.compare("true") == 0) {
      uncertainty_bound(0, 2) = 1e-4;
      uncertainty_bound(1, 2) = -1e-4;
      uncertainty_bound(2, 2) = 1e-5;
      uncertainty_bound(3, 2) = -1e-5;
    }

    return uncertainty_bound;
  }

};  // end class UCBoundNodelet
}  // namespace uc_bound

// Declare the plugin
PLUGINLIB_EXPORT_CLASS(uc_bound::UCBoundNodelet, nodelet::Nodelet);
