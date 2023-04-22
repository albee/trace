/**
 * @file backup_controller_nodelet.cc
 * @author MIT SSL
 * @brief An alternative backup PD controller.
 * @version 0.1
 * @date 2023-04-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <string.h>
#include <thread>
#include <string>
#include <fstream>
#include <Eigen/Dense>
#include <vector>
#include <iostream>

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// FSW includes
#include <ff_util/ff_flight.h>
#include <ff_util/ff_names.h>
#include <ff_util/ff_nodelet.h>
#include <ff_msg_conversions/ff_msg_conversions.h>
#include <ff_msgs/ControlState.h>
#include <ff_msgs/EkfState.h>
#include <ff_msgs/FamCommand.h>
#include <ff_msgs/FlightMode.h>


using namespace Eigen;

namespace backup_controller {

std::string TOPIC_STATE   = "gnc/ekf";
std::string TOPIC_TRAJ    = "gnc/ctl/setpoint/pd";
std::string TOPIC_FAM_CMD = "gnc/ctl/command";
std::string PARAM_CONTROL = "td/controller";

class BackupControllerNodelet : public ff_util::FreeFlyerNodelet{
 public:
  BackupControllerNodelet() : ff_util::FreeFlyerNodelet(true) {}
  ~BackupControllerNodelet() {}

 private:
  std::shared_ptr<std::thread> thread_;

  ros::Subscriber sub_state_;
  ros::Subscriber sub_traj_;
  ros::Publisher  pub_fam_cmd_;

  // DEBUGGING
  // FLIGHT MODE
  ros::Publisher pub_flight_mode_;
  ff_msgs::FlightMode flight_mode_;
  std::string flight_mode_name_ = "nominal";  // FlightMode to enter
  // END FLIGHT MODE

  ros::Publisher  pub_debug_;

  std::string controller_;  // = "default"; // = "pd";  // controller to send
                            // commands to

  ff_msgs::EkfState     state;
  ff_msgs::ControlState traj;
  ff_msgs::ControlState error;
  ff_msgs::ControlState d_error;
  ff_msgs::ControlState error_log;
  ff_msgs::FamCommand   fam_iss;
  ff_msgs::FamCommand   fam_bee;

  // controller gains
  double k_p_pos = 0;
  double k_d_pos = 0;

  double k_p_vel = 0;
  double k_d_vel = 0;

  double k_p_acc = 0;
  double k_d_acc = 0;

  double k_p_orient = 0.2;
  double k_d_orient = 0;

  void Initialize(ros::NodeHandle* nh) {
    // FLIGHT MODE
    if (!ff_util::FlightUtil::GetFlightMode(flight_mode_, flight_mode_name_)) {
      return;
    }  // create a nominal FlightMode
    pub_flight_mode_ = nh->advertise<ff_msgs::FlightMode>(TOPIC_MOBILITY_FLIGHT_MODE, 1, true);  // FlightMode
    sleep(5);
    pub_flight_mode_.publish(flight_mode_);  //
    // END FLIGHT MODE

    // subscribers & publishers
    sub_state_ = nh->subscribe<ff_msgs::EkfState>(TOPIC_STATE,
                                                  5,
                                                  boost::bind(&BackupControllerNodelet::stateCallback, this, _1));
    // need to boost::bind your subscriber callback---this is a nodelet
    // requirement
    sub_traj_  = nh->subscribe<ff_msgs::ControlState>(TOPIC_TRAJ,
                                                  5,
                                                  boost::bind(&BackupControllerNodelet::trajCallback, this, _1));
    pub_fam_cmd_ = nh->advertise<ff_msgs::FamCommand>(TOPIC_GNC_CTL_COMMAND,
                                                  5,
                                                  true);
    pub_debug_ = nh->advertise<geometry_msgs::Quaternion>("gnc/ctl/debug",
                                                  5,
                                                  true);
    NODELET_INFO_STREAM("[BACKUP CONTROLLER] Initialized");
    thread_.reset(new std::thread(&backup_controller::BackupControllerNodelet::Run, this));
  }  // Initialize()

  void stateCallback(const ff_msgs::EkfState::ConstPtr state_msg) {
    state = *state_msg;
  }

  void trajCallback(const ff_msgs::ControlState::ConstPtr traj_msg) {
    traj = *traj_msg;
  }


  void Run() {
    ros::Rate rate(2.0);
    while (ros::ok()) {
      rate.sleep();
      ros::param::getCached(PARAM_CONTROL, controller_);
      if (controller_.compare("pd") == 0) {
        controller_main();
      }
      ros::spinOnce();
    }
  }  // Run()

  void controller_main() {
    get_error();
    // get_d_error();
    get_cmd();
    rotate_frame();

    pub_fam_cmd_.publish(fam_bee);
    error_log = error;

    // DEBUGGING
    // pub_debug_.publish(error);
  }

  // ISS frame error
  void get_error() {
    // look for shorthand to subtract vectors
    // error.pose.position = vector_subtract(traj.pose.position, state.pose.position);
    error.when = ros::Time::now();
    geometry_msgs::Quaternion q_debug;

    tf2::Quaternion q_act_inrt;  // Actual orientation, inertial frame
    tf2::Quaternion q_des_inrt;  // Desired orientation, inertial rame
    tf2::Quaternion
        q_act_body;  // Actual orientation, body frame (to be computed)
    tf2::Quaternion
        q_des_body;  // Desired orientation, body frame (to be computed)
    tf2::Quaternion
        q_err_body;  // Orientation error in body frame (to be computed)

    tf2::convert(state.pose.orientation,
                 q_act_inrt);  // Convert Astrobee callbacks to tf2::Quaternions
    tf2::convert(traj.pose.orientation,  q_des_inrt);

    q_act_body = q_act_inrt.inverse() *
                 q_act_inrt;  // Rotate actual orientation, inertial to body
    q_des_body = q_act_inrt.inverse() *
                 q_des_inrt;  // Rotate desired orientation, inertial to body
    q_act_body.normalize();
    q_des_body.normalize();

    q_err_body = q_des_body * q_act_body.inverse();
    q_err_body.normalize();
    tf2::convert(q_err_body, error.pose.orientation);

    // Alternate: Compute inertial error, convert error to body frame
    // tf2::Quaternion q_err_inrt;
    // q_err_inrt = q_des_inrt * q_act_inrt.inverse();
    // q_err_body = q_act_inrt.inverse() * q_err_inrt;
    // tf2::convert(q_err_body, error.pose.orientation);

    // DEBUGGING
    geometry_msgs::Vector3 v_debug;
    double r, p, y;
    tf2::Matrix3x3(q_err_body).getRPY(r, p, y);
    v_debug.x = r;
    v_debug.y = p;
    v_debug.z = y;
    tf2::convert(q_des_body, q_debug);
    pub_debug_.publish(q_debug);
    // END DEBUGGING

    error.twist.angular.x =
        traj.twist.angular.x - state.omega.x;  // angular rate errors
    error.twist.angular.y = traj.twist.angular.y - state.omega.y;
    error.twist.angular.z = traj.twist.angular.z - state.omega.z;

    tf2::Vector3 omega_iss;
    tf2::Vector3 omega_bee;
    tf2::convert(error.twist.angular, omega_iss);
    omega_bee = tf2::Transform(q_act_inrt.inverse()) * omega_iss;
    tf2::convert(omega_bee, error.twist.angular);
  }

  void get_d_error() {
    double dt;
    double t_start = error_log.when.toSec();
    double t_end = error.when.toSec();
    dt = t_end - t_start;
  }

  // Use gains and error to calculate force and torque commands
  void get_cmd() {
    // Already in the correct frame
    fam_bee.wrench.torque.x = 2*k_p_orient*error.pose.orientation.x + k_d_orient*error.twist.angular.x;
    fam_bee.wrench.torque.y = 2*k_p_orient*error.pose.orientation.y + k_d_orient*error.twist.angular.y;
    fam_bee.wrench.torque.z = 2*k_p_orient*error.pose.orientation.z + k_d_orient*error.twist.angular.z;
  }

  // Rotate the commands from ISS frame to robot frame
  void rotate_frame() {
    tf2::Quaternion q_state;  // initial
    // tf2::Quaternion q_traj;   //pose.orientation final
    tf2::Vector3 force_iss;
    tf2::Vector3 force_bee;
    tf2::convert(state.pose.orientation, q_state);
    tf2::convert(fam_iss.wrench.force, force_iss);
    force_bee =
        tf2::Transform(q_state.inverse()) *
        force_iss;  // Multiplies matrix by ISS-frame vector, gets robot-frame
    tf2::convert(force_bee, fam_bee.wrench.force);
  }
};  // BackupController
}  // namespace backup_controller

PLUGINLIB_EXPORT_CLASS(backup_controller::BackupControllerNodelet, nodelet::Nodelet);
