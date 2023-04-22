/**
 * @file csv_read.cc
 * @author Keenan Albee
 * @brief Read in CSV files in standard motion planner format.
 * @version 0.1
 * @date 2023-04-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "data/csv_read.h"

using namespace Eigen;
using namespace std;

namespace csv {

// ISS start position set from sim_td.launch. Used for target .dat trajectories
// TODO: this should not be manually set (but works for TS2)!
std::vector<double> start_pose_target_ISS{10.9, -6.65, 4.9, 0.7071068, 0, 0, -0.7071068};
std::vector<double> start_pose_chaser_ISS{10.9, -8.15, 4.9, 0.7071068, 0, 0, 0.7071068};
std::vector<double> start_pose_target_ground{0.0, -0.5, -0.7, 1, 0, 0, 0};
std::vector<double> start_pose_chaser_ground{0.0, 0.6, 0.7, 0.7071068, 0, 0, -0.7071068};

/* Reads in a CSV and produces an Eigen matrix (specify type M) ). Credit to Stackoverflow answer.
   Input:
   path: string of csv file location
*/
template<typename M>
M load_csv (const std::string& path) {
    std::ifstream indata;
    indata.open(path);
    std::string line;
    std::vector<double> values;
    uint rows = 0;
    while (std::getline(indata, line)) {
        std::stringstream lineStream(line);
        std::string cell;
        while (std::getline(lineStream, cell, ',')) {
            values.push_back(std::stod(cell));
        }
        ++rows;
    }
    return Map<const Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, RowMajor>>(values.data(), rows, values.size()/rows);
}


/* Read in the standard-formatted output directory specifically for the Target.
   Input:
   path: string to the LUT DATA directory, containing .dat files.
*/
template<typename M>
M load_dat_target (const std::string& path) {
    std::ifstream indata;
    indata.open(path);
    std::string line;
    std::vector <double> values;
    uint rows = 0;

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

    MatrixXd data_matrix = Map<const Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, RowMajor>>(values.data(), rows, values.size()/rows);
    // 0: - Time
    // 1-3: position
    // 4-6: linear velocity
    // 7-10: orientation
    // 11-13: angular velocity
    // 14-16: linear accelaration
    // 17-19: angular accelaraion
    MatrixXd formatted_matrix(rows, 20);

    // .dat Target Trajectory Formatting
    for (int i = 0; i < 3; i++) {  // column index
      for (uint j = 0; j < rows; j++) {   // row index
        formatted_matrix(j,i+1)  = start_pose_target_ISS[i];  // set position
        formatted_matrix(j,i+4)  = 0.0;  // set linear velocity
        formatted_matrix(j,i+14) = 0.0;  // set linear accelaration
        formatted_matrix(j,i+17) = 0.0;  // set angular accelaration
      }
    }

    formatted_matrix.col(0) = data_matrix.col(0);  // set time
    for (int i = 0; i < 7; i++) {
      formatted_matrix.col(i+7) = data_matrix.col(i+1);  // set orientation and angular velocity
    }

    return formatted_matrix;
}  // end load_dat_target


/* Read in the standard-formatted output directory for the Chaser.
   Input:
   path: string to the LUT DATA directory, containing .dat files.
   TODO: deprecated!
*/
template<typename M>
M load_dat_chaser (const std::string& path) {

  // cout << "path input: " << path << endl;
  std::string FILE_POS  = path + "Results.dat";  // ?
  std::string FILE_VEL  = path + "Results_vel.dat";
  // std::string FILE_ORNT = path + "Results_";  // Missing data for now
  // std::stromg FILE_OMEG = path + "Results_";
  std::string FILE_ACC  = path + "Results_acc.dat";
  std::string FILE_JERK = path + "Results_jerk.dat";

  std::vector<std::string> filepaths{FILE_POS, FILE_VEL, FILE_ACC, FILE_JERK};
  uint rows = 0;
  std::vector <double> values;  // store data from file as parsed
  std::vector <MatrixXd> raw_data;  // store matrix data from each file


  for (int i = 0; i < 4; i++) {
    // cout << filepaths[i] << endl;
    std::string file = filepaths[i];
    rows = 0;
    std::ifstream indata;
    indata.open(file);
    std::string line;

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
    MatrixXd data_matrix = Map<const Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, RowMajor>>(values.data(), rows, values.size()/rows);
    raw_data.push_back(data_matrix);
    values.clear();
    // cout << rows << endl;
  }

  MatrixXd formatted_matrix(rows, 20);

  // Hardcode angular information (no current data)
  for (int i = 0; i < 4; i++) {
    for (uint j = 0; j < rows; j++) {
      formatted_matrix(j,7+i) = start_pose_chaser_ISS[3+i];  // orientation
      if (i != 3) {  // only needs to iterate 3 times
        formatted_matrix(j,11+i) = 0.0;  // angular velocity
        formatted_matrix(j,17+i) = 0.0;  // angular accelaration
      }
    }
  }

  // Transfer data from file read matrices to output matrix
  formatted_matrix.col(0) = raw_data[0].col(0);  // extract time from first file, they're all the same
  for (int i = 1; i < 4; i++) {  // position data
    formatted_matrix.col(i)    = raw_data[0].col(i);  // position data
    formatted_matrix.col(3+i)  = raw_data[1].col(i);  // linear velocity data
    formatted_matrix.col(13+i) = raw_data[2].col(i);  // linear accelaration data
    // formatted_matrix.col(19+i) = raw_data[3].col(i);  // jerk data (not supported in ControlState.msg)
  }

  return formatted_matrix;
}  // end load_dat_chaser
}  // end namespace csv

// explicit instantiation of required templates
template MatrixXd csv::load_csv<MatrixXd>(const std::string& path);
template MatrixXd csv::load_dat_target<MatrixXd>(const std::string& path);
template MatrixXd csv::load_dat_chaser<MatrixXd>(const std::string& path);
