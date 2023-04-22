#ifndef BACKUP_CONTROLLER_H_
#define BACKUP_CONTROLLER_H_

#include <Eigen/Dense>
#include <ff_msgs/FamCommand.h>
#include <ff_msgs/EkfState.h>
#include <ff_msgs/ControlState.h>
#include <ff_msgs/FlightMode.h>
using namespace Eigen;

namespace pd {
  class BackupController {
   public:
    BackupController();
    ~BackupController();

    // controller inputs and outputs
    ff_msgs::ControlState error;    // pos error for feedback
    ff_msgs::ControlState d_error;  // vel error for feedback
    ff_msgs::ControlState error_log;

    // controller gains
    double k_p_orient;
    double k_d_orient;

    Vector3d controller_main(ff_msgs::EkfState state, ff_msgs::ControlState x_des, Matrix<double, 3, 1> torque_mag);
    void get_error(ff_msgs::EkfState state, ff_msgs::ControlState x_des);
    Vector3d get_cmd(Matrix<double, 3, 1> torque_mag);
    Vector3d scale_torque_vector(Matrix<double, 3, 1> torque_mag, double torque_x, double torque_y, double torque_z);
  };
}

#endif  // BACKUP_CONTROLLER_H_
