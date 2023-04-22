%% Creates an output folder in Caroline's format.
% dt - timestep
% tf - final time
% REF_TRAJ - ref traj name

dt = 0.2;
tf = 30.0;
WRITE_PATH = '../../input/';
REF_TRAJ = 'MIT-ISS';  % {'SINUSOID', 'STEPX', 'STEPY', 'STEPZ', 'ZERO', 'ATTITUDE_X_STEPS'...
                                 % 'ATTITUDE_Y_STEPS', 'ATTITUDE_Z_STEPS', 'ATTITUDE_STEPS',
                                 % 'TEST3-GND' 'TEST3-ISS', 'MIT-ISS'}
% START_POS = [10.9; -9.65; 4.9];  % start position xyz
START_ATT = [0.0; 0.0; 0.7071; -0.7071];  % start atttiude, [qx qy qz qw] 
START_POS = [10.9; -8.15; 4.9];
% START_POS = [10.9; -9.65; 4.9];
% START_POS = [0; 0.6; -0.7];
NAME = REF_TRAJ;

% create the ref traj
[des_traj, u_des] = create_des_traj(dt, tf, REF_TRAJ, START_POS, START_ATT);

% put the ref traj in Caroline format
des_traj_to_caroline_format(WRITE_PATH, des_traj, u_des, NAME);

%{
output_x: [
  t x y z xd yd zd qx qy qz qw wx wy wz xdd ydd zdd wxd wyd wzd
  ...
]

output_u: [
  t ux uy uz
  ...
]

Files:
  results_pos_0.dat, results_vel_0.dat, results_ori_0.dat, results_ome_0.dat
  results_input_0.dat
%}

%% Reads in a directory of Caroline output format
% PATH = '../../motion_planner_interface/bin/ground-traj1-public';
PATH = strcat('../../input/', REF_TRAJ);
[x, v, quat, omega] = read_in_caroline_format(PATH);

%% Visualizes a Caroline output format
addpath('./plotting');
anim_tumble(x, quat, x);