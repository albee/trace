/**
 * @file motion_planner_interface_nodelet.cc
 * @author Keenan Albee
 * @brief A nodelet to assemble all inputs needed to call a motion planner.
 * @version 0.1
 * @date 2023-04-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <Eigen/Dense>
#include <vector>
#include <fstream>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>

// ROS
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include <geometry_msgs/Inertia.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>

// FSW
#include <ff_util/ff_nodelet.h>
#include <ff_util/ff_names.h>
#include <ff_util/ff_flight.h>
#include <ff_msgs/FamCommand.h>
#include <ff_msgs/ControlState.h>
#include <ff_msgs/FlightMode.h>
#include <ff_msgs/EkfState.h>
#include <ff_msg_conversions/ff_msg_conversions.h>

// TRACE
#include "mpdebugmsg.h"
#include <trace_msgs/TDStatus.h>
#include <trace_msgs/TDMotionPlannerInterfaceStatus.h>
#include "data/att_utils.h"
#include "data/eigen_msg.h"
#include "data/eigen_kdl.h"
#include "data/traj_utils.h"

namespace motion_planner_interface {

class MotionPlannerInterfaceNodelet : public ff_util::FreeFlyerNodelet {
 public:
  MotionPlannerInterfaceNodelet() : ff_util::FreeFlyerNodelet(true) {}
  ~MotionPlannerInterfaceNodelet() {}

 private:
  std::string TRAJ_PATH_ = "./";
  bool SIM_ = false;
  bool GROUND_ = false;

  int test_number_ = 0;

  int LUT_param_ = 4;  // set by chaser_coordinator
  int predef_LUT_param_ = 4;  // predefined by YAML
  int LUT_param_0_ = 4;  // actually sent to mpMIT

  bool activate_ = false;
  bool motion_plan_finished_ = false;
  bool motion_plan_published_ = false;
  std::string mpMIT_call_ = "";

  // Subscribers
  ros::Subscriber sub_chaser_pose_;
  ros::Subscriber sub_chaser_twist_;
  ros::Subscriber sub_target_pose_;
  ros::Subscriber sub_target_twist_;
  ros::Subscriber sub_status_;
  ros::Subscriber sub_inertia_;

  // Publishers
  ros::Publisher pub_chaser_traj_pose0_;
  ros::Publisher pub_chaser_traj_twist0_;
  ros::Publisher pub_target_traj_pose0_;
  ros::Publisher pub_target_traj_twist0_;
  ros::Publisher pub_debug_numbers_;
  ros::Publisher pub_debug_string_;
  ros::Publisher pub_debug_violations_;
  ros::Publisher pub_motion_planner_interface_status_;

  // State holders
  struct State {
    Eigen::Vector3f pos;
    Eigen::Vector3f vel;
    Eigen::Vector4f quat;
    Eigen::Vector3f omega;
  };

  State chaser_est_state_;
  State target_est_state_;
  State chaser_est_state_predef_;
  State target_est_state_predef_;
  State chaser_traj_state_0_;
  State target_traj_state_0_;

  // More parameters for motion planner
  Eigen::Matrix3f target_J_;
  Eigen::Matrix3f chaser_J_;
  float chaser_m_;

  float loop_rate_ = 2;

  // for subscriber management
  std::shared_ptr<std::thread> thread_;

  // This is called when the nodelet is loaded into the nodelet manager
  void Initialize(ros::NodeHandle* nh) {
    //get SIM_ and GROUND_ flags
    std::string sim;
    ros::param::getCached("/td/sim", sim);
    if (sim.compare("true") == 0) {
      SIM_ = true;
    }

    std::string ground;
    ros::param::getCached("/td/ground", ground);
    if (ground.compare("true") == 0) {
      GROUND_ = true;
    }

    // publishers and subscribers
    sub_chaser_pose_ = nh->subscribe<geometry_msgs::PoseWithCovariance>("/td/mit_slam/chaser_pose", 5, &MotionPlannerInterfaceNodelet::chaserPoseCallback, this);
    sub_chaser_twist_ = nh->subscribe<geometry_msgs::TwistWithCovariance>("/td/mit_slam/chaser_twist", 5, &MotionPlannerInterfaceNodelet::chaserTwistCallback, this);
    sub_target_pose_ = nh->subscribe<geometry_msgs::PoseWithCovariance>("/td/mit_slam/target_pose", 5, &MotionPlannerInterfaceNodelet::targetPoseCallback, this);
    sub_target_twist_ = nh->subscribe<geometry_msgs::TwistWithCovariance>("/td/mit_slam/target_twist", 5, &MotionPlannerInterfaceNodelet::targetTwistCallback, this);
    sub_inertia_ = nh->subscribe<geometry_msgs::Inertia>("/td/mit_slam/inertia", 5, &MotionPlannerInterfaceNodelet::inertiaCallback, this);
    sub_status_ = nh->subscribe<trace_msgs::TDStatus>("td/status", 5, &MotionPlannerInterfaceNodelet::TDStatusCallback, this);

    pub_chaser_traj_pose0_ = nh->advertise<geometry_msgs::PoseWithCovariance>("/td/motion_planner_interface/chaser_traj_pose0", 5, true);
    pub_chaser_traj_twist0_ = nh->advertise<geometry_msgs::TwistWithCovariance>("/td/motion_planner_interface/chaser_traj_twist0", 5, true);
    pub_target_traj_pose0_ = nh->advertise<geometry_msgs::PoseWithCovariance>("/td/motion_planner_interface/target_traj_pose0", 5, true);
    pub_target_traj_twist0_ = nh->advertise<geometry_msgs::TwistWithCovariance>("/td/motion_planner_interface/target_traj_twist0", 5, true);
    pub_motion_planner_interface_status_ = nh->advertise<trace_msgs::TDMotionPlannerInterfaceStatus>("/td/motion_planner_interface/status", 5, true);

    pub_debug_numbers_= nh->advertise<std_msgs::Float64MultiArray>("/td/motion_planner_interface/mpdebug_numbers", 5, true);
    pub_debug_string_ = nh->advertise<std_msgs::String>("/td/motion_planner_interface/mpdebug_path", 5, true);
    pub_debug_violations_=nh->advertise<std_msgs::Float64MultiArray>("/td/motion_planner_interface/mpdebug_violations", 5, true);

    // these are YAML values for test5 only
    std::vector<double> chaser_pos_ros;
    std::vector<double> target_pos_ros;
    std::vector<double> target_att_ros;
    std::vector<double> target_omega_ros;
    if (GROUND_) {
      ros::param::getCached("/td/motion_planner_interface/predef_chaser_pos_ground", chaser_pos_ros);
      ros::param::getCached("/td/motion_planner_interface/predef_target_pos_ground", target_pos_ros);
      ros::param::getCached("/td/motion_planner_interface/predef_target_att_ground", target_att_ros);
      ros::param::getCached("/td/motion_planner_interface/predef_target_omega_ground", target_omega_ros);
    }
    else {
      ros::param::getCached("/td/motion_planner_interface/predef_chaser_pos_iss", chaser_pos_ros);
      ros::param::getCached("/td/motion_planner_interface/predef_target_pos_iss", target_pos_ros);
      ros::param::getCached("/td/motion_planner_interface/predef_target_att_iss", target_att_ros);
      ros::param::getCached("/td/motion_planner_interface/predef_target_omega_iss", target_omega_ros);
    }

    // now populate the predefs
    chaser_est_state_predef_.pos << chaser_pos_ros[0], chaser_pos_ros[1], chaser_pos_ros[2];
    chaser_est_state_predef_.vel << 0.0, 0.0, 0.0;
    chaser_est_state_predef_.quat << 0.0, 0.0, 0.0, 1.0;
    chaser_est_state_predef_.omega << 0.0, 0.0, 0.0;

    target_est_state_predef_.pos << target_pos_ros[0], target_pos_ros[1], target_pos_ros[2];
    target_est_state_predef_.vel << 0.0, 0.0, 0.0;
    target_est_state_predef_.quat << target_att_ros[0], target_att_ros[1], target_att_ros[2], target_att_ros[3];
    target_est_state_predef_.omega << target_omega_ros[0], target_omega_ros[1], target_omega_ros[2];

    ros::param::getCached("/td/motion_planner_interface/predef_LUT_param", predef_LUT_param_);

    // Nominal target inertia
    std::vector<double> J_vec_target;
    ros::param::getCached("/td/motion_planner_interface/target_J", J_vec_target);
    int k = 0;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
          target_J_(i,j) = J_vec_target[k];
          k++;
      }
    }

    // Nominal chaser inertia
    std::vector<double> J_vec_chaser;
    ros::param::getCached("/td/motion_planner_interface/chaser_J", J_vec_chaser);
    k = 0;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
          chaser_J_(i,j) = J_vec_chaser[k];
          k++;
      }
    }

    // Nominal chaser mass
    ros::param::getCached("/td/motion_planner_interface/chaser_m", chaser_m_);

    NODELET_INFO_STREAM("[MotionPlannerInterface] Initialized.");

    thread_.reset(new std::thread(&motion_planner_interface::MotionPlannerInterfaceNodelet::Run, this));
  }

  void Run() {
    /* Check until activated, then go into planning mode
    */
    ros::Rate spin_rate(loop_rate_);
    while (ros::ok()) {
      if (activate_ == true) {
        if (test_number_ == 5) {
          bool use_predef = true;
          begin_plan_sequence(use_predef);
        }
        else {
          bool use_predef = false;
          begin_plan_sequence(use_predef);
        }
      }
      ros::spinOnce();
      spin_rate.sleep();
    }
  }

  int begin_plan_sequence(bool use_predef) {
    /* Perform motion planning with values from the SLAM.
    Wait until activated by chaser_coordinator. Then call motion planning executable/library based on the current state estimate.
    */
    ros::Rate spin_rate(loop_rate_);

    while (ros::ok()) {
      if (activate_ && !motion_plan_finished_) {
        if (use_predef == true) {  // use predef values
          chaser_traj_state_0_ = chaser_est_state_predef_;
          target_traj_state_0_ = target_est_state_predef_;
          LUT_param_0_ = predef_LUT_param_;
        }
        else {  // use SLAM
          chaser_traj_state_0_ = chaser_est_state_;
          target_traj_state_0_ = target_est_state_;
          LUT_param_0_ = LUT_param_;
        }

        motion_plan_finished_ = motionPlan();
      }

      if (motion_plan_finished_ && !motion_plan_published_) {
        publishMotionPlanInfo();
        motion_plan_published_ = true;

        trace_msgs::TDMotionPlannerInterfaceStatus msg;
        msg.stamp = ros::Time::now();
        msg.motion_plan_finished = motion_plan_finished_;
        msg.mpMIT_call = mpMIT_call_;
        pub_motion_planner_interface_status_.publish(msg);
      }

      ros::spinOnce();
      spin_rate.sleep();
    }
    return 1;
  }

  bool motionPlan() {
    /* _0_ values get used for motion planning.
    */
    NODELET_INFO_STREAM("[MotionPlannerInterface]: Chaser traj state 0 pos: \n" << chaser_traj_state_0_.pos);
    NODELET_INFO_STREAM("[MotionPlannerInterface]: Target traj state 0 quat: \n" << target_traj_state_0_.quat);

    // Call motion planner with the relevant inputs
    std::vector<int> params{1, 1, LUT_param_0_};

    // adjust y for standard convention on x- and z-axis tests
    if (test_number_ > 100) {
      std::string test_number_str = std::to_string(test_number_);
      if (test_number_str[0] == '1') { // x
        chaser_traj_state_0_.pos(1) += 1.5;
      }
      else if (test_number_str[0] == '3') { // z
        chaser_traj_state_0_.pos(1) += 1.5;
      }
    }

    std::vector<double> chaser_pos{(double)chaser_traj_state_0_.pos(0),
                                   (double)chaser_traj_state_0_.pos(1),
                                   (double)chaser_traj_state_0_.pos(2)};
    double chaser_mass = (double)chaser_m_;
    std::vector<double> target_inertia;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
          target_inertia.push_back(double(target_J_(i,j)));
      }
    }
    std::vector<double> target_pos{(double)target_traj_state_0_.pos(0),
                                   (double)target_traj_state_0_.pos(1),
                                   (double)target_traj_state_0_.pos(2)};
    std::vector<double> target_quat{(double)target_traj_state_0_.quat(0),
                                    (double)target_traj_state_0_.quat(1),
                                    (double)target_traj_state_0_.quat(2),
                                    (double)target_traj_state_0_.quat(3)};
    std::vector<double> target_ang_vel{(double)target_traj_state_0_.omega(0),
                                       (double)target_traj_state_0_.omega(1),
                                       (double)target_traj_state_0_.omega(2)};
    ros::param::getCached("/td/chaser_traj_file", TRAJ_PATH_);

    // call motion planner
    NODELET_INFO_STREAM("[MotionPlannerInterface]: Planner executable call reached");
    try {
      mpMIT_call_ = traj_utils::call_planner(params, chaser_pos, chaser_mass, target_inertia, target_pos, target_quat, target_ang_vel, TRAJ_PATH_, SIM_, GROUND_);
    }
    catch (...) {
      NODELET_ERROR_STREAM("[MotionPlannerInterface]: Planner executable call failed! Call errored.");
      return false;
    }

    // check that output was produced
    std::ifstream indata;
    std::string time_file = TRAJ_PATH_ + "result_time_0.dat";
    indata.open(time_file);
    if(indata.fail()){
      NODELET_ERROR_STREAM("[MotionPlannerInterface]: Planner executable call failed! No output.");
      return false;
    }

    return true;  // finished succesfully
  }

  void publishMotionPlanInfo() {
    // Publish initial chaser and target trajectory states for UC bound usage
    publishChaserPose(chaser_traj_state_0_.pos, chaser_traj_state_0_.quat);
    publishChaserTwist(chaser_traj_state_0_.vel, chaser_traj_state_0_.omega);
    publishTargetPose(target_traj_state_0_.pos, target_traj_state_0_.quat);
    publishTargetTwist(target_traj_state_0_.vel, target_traj_state_0_.omega);

    // Read in files for messages
    std::ifstream debugfile;
    std::string readinname;

    // mp_num.dat
    readinname = TRAJ_PATH_ + "mp_num.dat";
    double nvals;
    Eigen::Matrix<double,194,1> nreadin = Eigen::Matrix<double, 194, 1>::Zero();
    int countin=0;
    debugfile.open(readinname);
    std::vector<double> valsin;
    if (debugfile){
      while (debugfile>>nvals){
          nreadin(countin) = nvals;
          countin++;
      }
    }
    debugfile.close();

    // mp_str.dat
    readinname = TRAJ_PATH_ + "mp_str.dat";
    std::string svals;
    debugfile.open(readinname);
    std::getline(debugfile, svals);
    debugfile.close();

    // mp_vio.dat
    readinname = TRAJ_PATH_ + "mp_vio.dat";
    double vvals;
    std::vector<double> vreadin;
    debugfile.open(readinname);
    if (debugfile) {
      while (debugfile>>vvals) {
          vreadin.push_back(vvals);
      }
    }
    debugfile.close();

    int sizevios = vreadin.size();
    Eigen::Matrix<double, Eigen::Dynamic, 1> vvreadin = Eigen::MatrixXd::Zero(sizevios, 1);

    for(int counter = 0; counter < sizevios; counter++) {
      vvreadin(counter) = vreadin[counter];
    }

    // publish!
    std_msgs::Float64MultiArray nreadout;
    tf::matrixEigenToMsg(nreadin, nreadout);
    pub_debug_numbers_.publish(nreadout);

    std_msgs::String msg;
    msg.data = svals;
    pub_debug_string_.publish(msg);

    std_msgs::Float64MultiArray vreadout;
    tf::matrixEigenToMsg(vvreadin, vreadout);
    pub_debug_violations_.publish(vreadout);
  }

  void publishChaserPose(const Eigen::Vector3f pos, const Eigen::Vector4f quat) {
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

    pub_chaser_traj_pose0_.publish(p);
  }

  void publishTargetPose(const Eigen::Vector3f pos, const Eigen::Vector4f quat) {
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

    pub_target_traj_pose0_.publish(p);
  }

  void publishChaserTwist(const Eigen::Vector3f vel, const Eigen::Vector3f omega) {
    geometry_msgs::TwistWithCovariance p;

    double pi = 3.14159265358979323846;

    p.twist.linear.x = vel(0);
    p.twist.linear.y = vel(1);
    p.twist.linear.z = vel(2);
    p.twist.angular.x = omega(0) * pi / 180;
    p.twist.angular.y = omega(1) * pi / 180;
    p.twist.angular.z = omega(2) * pi / 180;

    int idx = 0;
    for (int i = 0; i < 6; i++) {
      for (int j = 0; j < 6; j++) {
        p.covariance[idx] = 0.0;
        idx++;
      }
    }

    pub_chaser_traj_twist0_.publish(p);
  }

  void publishTargetTwist(const Eigen::Vector3f vel, const Eigen::Vector3f omega) {
    geometry_msgs::TwistWithCovariance p;

    double pi = 3.14159265358979323846;

    p.twist.linear.x = vel(0);
    p.twist.linear.y = vel(1);
    p.twist.linear.z = vel(2);

    // convert back to rad/s for it to be used by other things
    p.twist.angular.x = omega(0) * pi / 180;
    p.twist.angular.y = omega(1) * pi / 180;
    p.twist.angular.z = omega(2) * pi / 180;

    int idx = 0;
    for (int i = 0; i < 6; i++) {
      for (int j = 0; j < 6; j++) {
        p.covariance[idx] = 0.0;
        idx++;
      }
    }

    pub_target_traj_twist0_.publish(p);
  }

  void TDStatusCallback(const trace_msgs::TDStatus::ConstPtr& msg) {
    activate_ = msg->motion_planner_interface_activate;
    test_number_ = msg->test_number;
    LUT_param_ = msg->LUT_param;
  }

  void chaserPoseCallback(const geometry_msgs::PoseWithCovariance::ConstPtr& msg) {
    chaser_est_state_.pos(0) = msg->pose.position.x;
    chaser_est_state_.pos(1) = msg->pose.position.y;
    chaser_est_state_.pos(2) = msg->pose.position.z;
    chaser_est_state_.quat(0) = msg->pose.orientation.x;
    chaser_est_state_.quat(1) = msg->pose.orientation.y;
    chaser_est_state_.quat(2) = msg->pose.orientation.z;
    chaser_est_state_.quat(3) = msg->pose.orientation.w;
  }

  void chaserTwistCallback(const geometry_msgs::TwistWithCovariance::ConstPtr& msg) {
    double pi = 3.14159265358979323846;

    chaser_est_state_.vel(0) = msg->twist.linear.x;
    chaser_est_state_.vel(1) = msg->twist.linear.y;
    chaser_est_state_.vel(2) = msg->twist.linear.z;

    // convert to deg/s for motion planner usage
    chaser_est_state_.omega(0) = msg->twist.angular.x * 180 / pi;
    chaser_est_state_.omega(1) = msg->twist.angular.y * 180 / pi;
    chaser_est_state_.omega(2) = msg->twist.angular.z * 180 / pi;
  }

  void targetPoseCallback(const geometry_msgs::PoseWithCovariance::ConstPtr& msg) {
    target_est_state_.pos(0) = msg->pose.position.x;
    target_est_state_.pos(1) = msg->pose.position.y;
    target_est_state_.pos(2) = msg->pose.position.z;
    target_est_state_.quat(0) = msg->pose.orientation.x;
    target_est_state_.quat(1) = msg->pose.orientation.y;
    target_est_state_.quat(2) = msg->pose.orientation.z;
    target_est_state_.quat(3) = msg->pose.orientation.w;
  }

  void targetTwistCallback(const geometry_msgs::TwistWithCovariance::ConstPtr& msg) {
    double pi = 3.14159265358979323846;

    target_est_state_.vel(0) = msg->twist.linear.x;
    target_est_state_.vel(1) = msg->twist.linear.y;
    target_est_state_.vel(2) = msg->twist.linear.z;

    // convert to deg/s for motion planner usage
    target_est_state_.omega(0) = msg->twist.angular.x  * 180 / pi;
    target_est_state_.omega(1) = msg->twist.angular.y  * 180 / pi;
    target_est_state_.omega(2) = msg->twist.angular.z  * 180 / pi;
  }

  void inertiaCallback(const geometry_msgs::Inertia::ConstPtr& inertia_msg) {
    target_J_(0,0) = inertia_msg->ixx;
    target_J_(0,1) = inertia_msg->ixy;
    target_J_(0,2) = inertia_msg->ixz;
    target_J_(1,1) = inertia_msg->iyy;
    target_J_(1,2) = inertia_msg->iyz;
    target_J_(2,2) = inertia_msg->izz;
    target_J_(1,0) = target_J_(0,1);
    target_J_(2,1) = target_J_(1,2);
    target_J_(2,0) = target_J_(0,2);
  }

};  // MotionPlannerInterfaceNodelet
}  // namespace motion_planner_interface

// Declare the plugin
PLUGINLIB_EXPORT_CLASS(motion_planner_interface::MotionPlannerInterfaceNodelet, nodelet::Nodelet);
