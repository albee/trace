# data/input

This directory contains reference trajectories and other input information for the Chaser and Target.

Contents:

* sample-trajectories: Misc. sample trajectories, including ones used for Target tracking.

* casadi-functions: Functions used for CasADi graph creation.

* target-tumbles: Target tumble trajectory debug data.

---

## Standard trajectory input format (motion_planner_interface)

The final format is 8 .dat files:
* results_time_0.dat
* results_pos_0.dat
* results_vel_0.dat
* results_acc_0.dat
* results_ori_0.dat
* results_ome_0.dat
* results_vel_0.dat
* results_input_0.dat

The output of reading in these files (see `traj_utils.h`) is

output_x = Eigen::MatrixXd(
  t x y z xd yd zd qx qy qz qw wx wy wz xdd ydd zdd wxd wyd wzd
  ...
  )

[row, 20]

output_u = Eigen::MatrixXd(
  t ux uy uz
  ...
  )

[row, 4]

--