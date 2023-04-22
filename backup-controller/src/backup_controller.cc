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
#include <math.h>
#include <cmath>
#include <Eigen/Dense>
#include <vector>
#include <iostream>

// ROS
#include <tf/tf.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// FSW
#include <ff_util/ff_names.h>
#include <ff_util/ff_flight.h>
#include <ff_msg_conversions/ff_msg_conversions.h>
#include <ff_msgs/FamCommand.h>
#include <ff_msgs/EkfState.h>
#include <ff_msgs/ControlState.h>
#include <ff_msgs/FlightMode.h>

#include "backup_controller/backup_controller.h"


using namespace Eigen;

namespace pd {
  BackupController::BackupController() {
    k_p_orient = 0.2;  // used if no param file
    k_d_orient = 0.2;

    std::string ground = "false";

    ros::param::getCached("/td/ground", ground);

    if (ground.compare("true") == 0) {
      ros::param::getCached("/td/casadi_nmpc/k_p_orient_ground", BackupController::k_p_orient);
      ros::param::getCached("/td/casadi_nmpc/k_d_orient_ground", BackupController::k_d_orient);
    }
    else {
      ros::param::getCached("/td/casadi_nmpc/k_p_orient_iss", BackupController::k_p_orient);
      ros::param::getCached("/td/casadi_nmpc/k_d_orient_iss", BackupController::k_d_orient);
    }

  }

  BackupController::~BackupController() {}

  Vector3d BackupController::controller_main(ff_msgs::EkfState state, ff_msgs::ControlState x_des, Matrix<double, 3, 1> torque_mag) {
    /* Calculate u_torques, and perform torque vector saturation scaling.

    [1] B. Wie and P. M. Barb, “Quaternion feedback for spacecraft large angle maneuvers,” J. Guid. Control. Dyn., vol. 8, no. 3, pp. 360–365, 1985.

    Input:
    state from EKF (att is body wrt inertial frame, ang_vel is body frame), as msg
    desired state (att is body wrt inertial frame, ang_vel is body frame), as msg

    Output:
    torques in body frame
    */
    Vector3d u_torques;

    get_error(state, x_des);
    u_torques = get_cmd(torque_mag);
    // rotate_frame(state, u_torques);

    error_log = error;

    return u_torques;
  }

  void BackupController::get_error(ff_msgs::EkfState state, ff_msgs::ControlState x_des) {
    /* Get ISS frame position error

    Output:
    error, as message
    */
    error.when = ros::Time::now();

    tf2::Quaternion q_act; // Actual orientation, inertial frame
    tf2::Quaternion q_des; // Desired orientation, inertial rame
    tf2::Quaternion q_err; // Orientation error

    // Convert Astrobee callbacks to tf2::Quaternions
    tf2::convert(state.pose.orientation, q_act);
    tf2::convert(x_des.pose.orientation,  q_des);
    q_act.normalize();
    q_des.normalize();

    // attitude error
    // "q_err is simply the current spacecraft attitude quaternion w.r.t. the commanded attitide."
    //q_err = q_act * q_des.inverse();
    q_err = q_act.inverse() * q_des;
    if (q_err.getW() < 0) {
      q_err = q_err * -1;
    }
    q_err.normalize();

    // convert back to quaternion msg (error quaternion)
    tf2::convert(q_err, error.pose.orientation);

    // ang vel error (already in body frame: cur-des)
    error.twist.angular.x = state.omega.x - x_des.twist.angular.x; // angular rate errors
    error.twist.angular.y = state.omega.y - x_des.twist.angular.y;
    error.twist.angular.z = state.omega.z - x_des.twist.angular.z;
  }

  Vector3d BackupController::get_cmd(Matrix<double, 3, 1> torque_mag) {
    /* The actual controller. Use gains and error to calculate force and torque commands.

    Inputs:
    torque_mag: [tx ty ty] max magnitudes (box constraint)
    quaternion error [x y z w]
    twist error [x y z], body frame
    */

    int sign = 1;
    if (error.pose.orientation.w < 0){
      sign = -1;
    }

    // Method 0: NASA's way of applying attitude/omega gains (weird quat convention)
    double torque_x = -k_d_orient * (-k_p_orient*error.pose.orientation.x + error.twist.angular.x);
    double torque_y = -k_d_orient * (-k_p_orient*error.pose.orientation.y + error.twist.angular.y);
    double torque_z = -k_d_orient * (-k_p_orient*error.pose.orientation.z + error.twist.angular.z);

    // std::cout << "xyz: " << torque_x << " " << torque_y << " " << torque_z << std::endl;

    Vector3d u_torques = scale_torque_vector(torque_mag, torque_x, torque_y, torque_z);

    // std::cout << "u_torques: " << u_torques << std::endl;

    // Method 1:
    //double torque_x = -2*k_p_orient*error.pose.orientation.x - k_d_orient*error.twist.angular.x;
    //double torque_y = -2*k_p_orient*error.pose.orientation.y - k_d_orient*error.twist.angular.y;
    //double torque_z = -2*k_p_orient*error.pose.orientation.z - k_d_orient*error.twist.angular.z;

    // Method 2:
    //double torque_x = -2*k_p_orient*error.pose.orientation.x / pow(error.pose.orientation.w,3) - k_d_orient*error.twist.angular.x;
    //double torque_y = -2*k_p_orient*error.pose.orientation.y / pow(error.pose.orientation.w,3) - k_d_orient*error.twist.angular.y;
    //double torque_z = -2*k_p_orient*error.pose.orientation.z / pow(error.pose.orientation.w,3) - k_d_orient*error.twist.angular.z;

    // Method 3:
    //double torque_x = -sign*2*k_p_orient*error.pose.orientation.x - k_d_orient*error.twist.angular.x;
    //double torque_y = -sign*2*k_p_orient*error.pose.orientation.y - k_d_orient*error.twist.angular.y;
    //double torque_z = -sign*2*k_p_orient*error.pose.orientation.z - k_d_orient*error.twist.angular.z;

    // Vector3d u_torques;
    // u_torques << torque_x, torque_y, torque_z;

    return u_torques;
  }

  Vector3d BackupController::scale_torque_vector(Matrix<double, 3, 1> torque_mag, double torque_x, double torque_y, double torque_z) {
    /* Scale torque vector to max magnitude. For now, conservatively scales down to lowest torque_mag magnitude.
    */
    Vector3d torque_scaled;

    // std::cout << "inputs: " << torque_mag << " " << torque_x << " " << torque_y << " " << torque_z << std::endl;

    double norm = sqrt(pow(torque_x, 2) + pow(torque_y, 2) + pow(torque_z, 2));
    double max_torque = torque_mag.minCoeff();
    double c = 1.0;  // scaling coefficient

    if (norm > max_torque) {
      // to be conservative, scale to c = smallest torque_mag / norm
      c = max_torque/norm;
    }
    torque_scaled << c*torque_x, c*torque_y, c*torque_z;

    return torque_scaled;
  }
}
