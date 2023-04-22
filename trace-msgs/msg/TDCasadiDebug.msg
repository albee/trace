std_msgs/Header header  # header with time stamp
geometry_msgs/Wrench wrench  # force and torque
geometry_msgs/Point u0_mpc  # nominal mpc computation
geometry_msgs/Point u0_dr  # disturbance rejection computation
std_msgs/Float64MultiArray x_nom  # selected x0
geometry_msgs/Vector3 accel  # linear acceleration (wrench w/out estimated mass)
std_msgs/Float64 casadi_comp_time
std_msgs/Float64 total_comp_time
std_msgs/String control_mode
std_msgs/String state_mode

# gains
float64 Q_pos_factor
float64 Q_vel_factor
float64 R_factor
float64 QN_pos_factor
float64 QN_vel_factor

float64 Q_pos_tube_factor
float64 Q_vel_tube_factor
float64 R_tube_factor
float64 QN_pos_tube_factor
float64 QN_vel_tube_factor

float64 Q_pos_anc_factor
float64 Q_vel_anc_factor
float64 R_anc_factor

# timing info
float64 T
int32 N
float64 control_dt
