/**
 * @brief Functions to call a motion planning executable and read in properly formatted data.
 * 
 */

#ifndef TRAJ_UTILS_H_
#define TRAJ_UTILS_H_

#include <iostream>
#include <stdlib.h>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>


namespace traj_utils {

extern std::vector<double> start_pose_target_ISS;
extern std::vector<double> start_pose_chaser_ISS;
extern std::vector<double> start_pose_target_ground;
extern std::vector<double> start_pose_chaser_ground;

std::string call_planner(std::vector<int> params, std::vector<double> chaser_pos, double target_mass,
  std::vector<double> target_inertia, std::vector<double> target_pos,
  std::vector<double> target_quat, std::vector<double> target_ang_vel, std::string TRAJ_PATH, bool SIM, bool GROUND); 

template<typename M>
M get_planner_output_u(const std::string& path);

template<typename M>
M get_planner_output_x(const std::string& path);

}  // end namespace traj_utils
#endif // TRAJ_UTILS_H_
