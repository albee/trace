/**
 * @file traj_utils.cc
 * @author Keenan Albee
 * @brief Script to call a motion planning excutable or read in standard-formatted plan data.
 * @version 0.1
 * @date 2023-04-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "data/traj_utils.h"


namespace traj_utils {

std::vector<double> start_pose_target_ISS{10.9, -6.65, 4.9, 0.7071068, 0, 0, -0.7071068};
std::vector<double> start_pose_chaser_ISS{10.9, -8.15, 4.9, 0.7071068, 0, 0, 0.7071068};
std::vector<double> start_pose_target_ground{0.0, -0.5, -0.7, 1, 0, 0, 0};
std::vector<double> start_pose_chaser_ground{0.0, 0.6, 0.7, 0.7071068, 0, 0, -0.7071068};

std::string call_planner(std::vector<int> params, std::vector<double> chaser_pos, double target_mass,
  std::vector<double> target_inertia, std::vector<double> target_pos,
  std::vector<double> target_quat, std::vector<double> target_ang_vel, std::string TRAJ_PATH, bool SIM, bool GROUND) {
  /*
  Call a motion planning executable with a specific format. For example,

  query number - set to 1 for now
  knots flag  - set to 1 for now
  scenario number - 1 through 18
  chaser initial pos x
  chaser initial pos y
  chaser initial pos z
  target mass
  target inertia 00
  target inertia 01
  target inertia 02
  target inertia 10
  target inertia 11
  target inertia 12
  target inertia 20
  target inertia 21
  target inertia 22
  target initial pos x
  target initial pos y
  target initial pos z
  target initial orientation q_x
  target initial orientation q_y
  target initial orientation q_z
  target initial orientation q_w
  target initial angular velocity x
  target initial angular velocity y
  target initial angular velocity z
  traj file name WITH trailing slash (string). Trailing slash is REMOVED before exe call.
  SIM: 0 if sim, 1 if hardware

  sample exe call format:
  ./mpMIT 1 1 4 0 0.6 0 7828.0 17023.3 397.1 -2171.4 397.1 124825.70 344.2 -2171.4 344.2 129112.2 0 -0.5 0 0 0 114.82 0 0 -4.5028 /opt/roam/mp/
  */

  /**********************************/
  std::string CMD;
  // simulation (where the exe is)
  if (SIM == true) {
    CMD = TRAJ_PATH;
  }
  // hardware testing (where the exe is)
  else {
    if (GROUND == true) {
      CMD = "/opt/roam/bin_ground/";
    }
    else {
      CMD = "/opt/roam/bin_iss/";
    }
  }
  /**********************************/

  std::string EXE_NAME{"mpMIT"};
  CMD += EXE_NAME;

  // TRAJ_PATH.pop_back() removes the trailing slash from the normal TRAJ_PATH variable.
  std::string TRAJ_PATH_WITHOUT_SLASH = TRAJ_PATH;
  TRAJ_PATH_WITHOUT_SLASH.pop_back();

  std::stringstream ss;
  ss << " " << params[0]  << " " << params[1] << " " << params[2]
     << " " << chaser_pos[0] << " " << chaser_pos[1] << " " << chaser_pos[2]
     << " " << target_mass
     << " " << target_inertia[0] << " " << target_inertia[1] << " " << target_inertia[2]
     << " " << target_inertia[3] << " " << target_inertia[4] << " " << target_inertia[5]
     << " " << target_inertia[6] << " " << target_inertia[7] << " " << target_inertia[8]
     << " " << target_pos[0] << " " << target_pos[1] << " " << target_pos[2]
     << " " << target_quat[0] << " " << target_quat[1] << " " << target_quat[2] << " " << target_quat[3]
     << " " << target_ang_vel[0] << " " << target_ang_vel[1] << " " << target_ang_vel[2] << " " << TRAJ_PATH_WITHOUT_SLASH;
  CMD += ss.str();

  std::cout << CMD <<std::endl;

  std::system(CMD.c_str());  // system takes a char*

  std::cout << "...motion planning call complete.\n(SIM is: " << SIM << ")" << std::endl;

  return CMD;
}

template<typename M>
M get_planner_output_u(const std::string& path) {
  /* Get motion planning input history, with the following format:
  results_input_0.data

  Input:
  path of .dat directory

  Output:
  Matrix of inputs

  [row, 20]

  output_u = Eigen::MatrixXd(
   t ux uy uz
   ...
   )

  [row, 4]
  */
  std::string FILE_INPUT  = path + "results_input_0.dat";
  std::vector<std::string> filepaths{FILE_INPUT};
  int NUM_FILES = int(filepaths.size());
  uint rows = 0;
  std::vector <double> values;  // store data from file as parsed
  std::vector <Eigen::MatrixXd> raw_data;  // store matrix data from each file

  // for each file, grab all the data and put it in an output Eigen::MatrixXd
  for (int i = 0; i < NUM_FILES; i++) {
    std::string file = filepaths[i];
    rows = 0;
    std::ifstream indata;
    indata.open(file);
    if(indata.fail()){
      std::cout << "Traj file read failed!" << std::endl;
    }
    std::string line;

  // Get raw data from each file; parse until no more lines
  while (std::getline(indata, line)) {
    std::stringstream lineStream(line);
    std::string cell;
    while (std::getline(lineStream, cell, ' ')) {
      if (!cell.empty()) {  // big ol bugs without this -- delimiter repeats an unspecified number of times before each value
        values.push_back(std::stod(cell));
      }
    }
    ++rows;
  }
   indata.close();
   Eigen::MatrixXd data_matrix = Eigen::Map<const Eigen::Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, Eigen::RowMajor>>(values.data(), rows, values.size()/rows);
   raw_data.push_back(data_matrix);
   values.clear();  // prepare for new set of data
  }
  Eigen::MatrixXd formatted_matrix(rows, 3); // [rows, 3] matrix.

  // Transfer data from file matrix to output matrix
  formatted_matrix.col(0) = raw_data[0].col(0);  // extract time from first column, they're all the same
  for (int j = 0; j < 3; j++) {  // position data (start at 1, since first column is always time)
    formatted_matrix.col(j).setZero();  // translational inputs, hardcoded
  }
  return formatted_matrix;
}

template<typename M>
M get_planner_output_x(const std::string& path){
  /*
  Gets motion planning executable output, with the following format:

  results_pos_0.dat, results_vel_0.dat, results_ori_0.dat, results_ome_0.dat

  The contents of
  these files appears in 4 columns: Time, x-dim, y-dim, z-dim. There are no headers.
  There are 301 rows of entries, corresponding to a 60 s maneuver sampled at 0.2 s.

  Input: absolute path of .dat files

  Output:

  output_x = Eigen::MatrixXd(
    t x y z xd yd zd qw qx qy qz wx wy wz xdd ydd zdd wxd wyd wzd
    ...
    )
  */
  std::string FILE_POS  = path + "results_pos_0.dat";
  std::string FILE_VEL  = path + "results_vel_0.dat";
  std::string FILE_QUAT = path + "results_ori_0.dat";
  std::string FILE_ANG_VEL = path + "results_ome_0.dat";

  //std::string FILE_ACC  = path + "Results_acc.dat";
  //std::string FILE_JERK = path + "Results_jerk.dat";

  std::vector<std::string> filepaths{FILE_POS, FILE_VEL, FILE_QUAT, FILE_ANG_VEL};
  int NUM_FILES = int(filepaths.size());
  uint rows = 0;
  std::vector <double> values;  // store data from file as parsed
  std::vector <Eigen::MatrixXd> raw_data;  // store matrix data from each file

  // for each file, grab all the data and put it in an output Eigen::MatrixXd
  for (int i = 0; i < NUM_FILES; i++) {
    std::string file = filepaths[i];
    rows = 0;
    std::ifstream indata;
    indata.open(file);
    if(indata.fail()){
      std::cout << "Traj file read failed!" << std::endl;
    }
    std::string line;

   // Get raw data from each file; parse until no more lines
   while (std::getline(indata, line)) {
     std::stringstream lineStream(line);
     std::string cell;
     // std::cout << lineStream.str() << '\n';
     while (std::getline(lineStream, cell, ' ')) {
       if (!cell.empty()) {  // big ol bugs without this -- delimiter repeats an unspecified number of times before each value
         values.push_back(std::stod(cell));
       }
     }
     ++rows;
  }
   indata.close();
   Eigen::MatrixXd data_matrix = Eigen::Map<const Eigen::Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, Eigen::RowMajor>>(values.data(), rows, values.size()/rows);
   raw_data.push_back(data_matrix);
   values.clear();  // prepare for new set of data
  }
  Eigen::MatrixXd formatted_matrix(rows, 20); // [rows, 20] matrix.

  // // Transfer data from file read matrices to output matrix
  formatted_matrix.col(0) = raw_data[0].col(0);  // extract time from first column, they're all the same
  for (int j = 0; j < 4; j++) {
    if (j < 3) {
      // formatted_matrix.col(j)    = raw_data[0].col(j+1);  // position data
      formatted_matrix.col(1+j)    = raw_data[0].col(j+1);  // position data
      formatted_matrix.col(4+j)  = raw_data[1].col(j+1);  // linear velocity data
      formatted_matrix.col(7+j)  = raw_data[2].col(j+1);  // Eigen::MatrixXd::Ones(rows, 1)*start_pose_chaser_ground[3+j]; orientation (quat)
      formatted_matrix.col(11+j) = raw_data[3].col(j+1);  // angular velocity
      formatted_matrix.col(14+j).setZero();  // raw_data[2].col(i);  // linear accelaration data, hardcoded
      formatted_matrix.col(17+j).setZero();  // angular accelaration, hardcoded
   }
   else {  // only for quat
     formatted_matrix.col(7+j) = raw_data[2].col(j+1); // orientation (quat), hardcoded
   }
  }

  return formatted_matrix;
}
}  // end namespace traj_utils

int main(int argc, char **argv) {
  /* A sample call of the executable.
  Modify relative path of .dat file input/output manually.
  */

  /**********************************/
  // simulation (where read/write happens)
  // std::string RELATIVE_PATH = "/bin/mpMIT-04-15-21-ground/";
  // std::string TRAJ_DIR_PATH = ros::package::getPath("motion_planner_interface");
  // TRAJ_DIR_PATH += RELATIVE_PATH;

  // hardware (where read/write happens)
  std::string TRAJ_DIR_PATH = "/opt/roam/mp/";  // must call with trailing slash
  /**********************************/

  std::cout << "Example of the executable call..." << std::endl;
  int LUT_param = 5;
  std::vector<int> params{1, 1, LUT_param};
  std::vector<double> chaser_pos{0.0, 0.6, 0.0};
  double target_mass = 7828.0;
  std::vector<double> target_inertia{17023.3, 397.1, -2171.4, 397.1, 124825.70, 344.2, -2171.4, 344.2, 129112.2};
  std::vector<double> target_pos{0.0, -0.5, 0.0};
  std::vector<double> target_quat{0.0, 0.0, 0.0, 1.0};  // is this currently a quat?
  std::vector<double> target_ang_vel{0.0, 0.0, 0.0};
  bool SIM = false;
  bool GROUND = false;

  std::cout << "input/ouput is at: " << TRAJ_DIR_PATH << std::endl;

  // make the call
  traj_utils::call_planner(params, chaser_pos, target_mass, target_inertia, target_pos, target_quat, target_ang_vel, TRAJ_DIR_PATH, SIM, GROUND);

  Eigen::MatrixXd output_x = traj_utils::get_planner_output_x<Eigen::MatrixXd>(TRAJ_DIR_PATH);
  Eigen::MatrixXd output_u = traj_utils::get_planner_output_u<Eigen::MatrixXd>(TRAJ_DIR_PATH);

  // std::cout << output_x << std::endl;
  // std::cout << output_u << std::endl;
}
