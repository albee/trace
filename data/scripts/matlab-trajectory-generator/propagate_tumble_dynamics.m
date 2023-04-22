% moments of inertia (principal coordinates)
I_xx = 3.0;
I_yy = 2.5;
I_zz = 5.0;

% inertia ratios
p_x = (I_yy - I_zz)/I_xx;
p_y = (I_zz - I_xx)/I_yy;
p_z = (I_xx - I_yy)/I_zz;

% quaternion
q_x0 = 0.0;
q_y0 = 0.0;
q_z0 = 0.0;
q_w0 = 1.0;
q_i = [q_x0; q_y0; q_z0; q_w0];

% angular velocity (body frame)
w_x0 = 0.0;%3.0;
w_y0 = 0.0;%0.01;
w_z0 = 0.0;%0.0;
w_i = [w_x0; w_y0; w_z0];
norm(w_i);

x_i = [q_i; w_i];

dt = 0.2;  % [s]
tf = 120.0;  % [s]

tumble_dynamics = casadi_6dof_tumble_dynamics(0);

log_state = [];

for i = 0:dt:tf
  log_state(end+1, :) = x_i';
  u_i = [0.05; 0; 0];
  
  x_i = full(tumble_dynamics(x_i, u_i, [p_x; p_y; p_z], dt));  % full is required to convert from casadi.DM    
end

%% Plotting
% ref traj, in body coordinates
r0_mat = zeros(size(log_state,1), 3);
R0_mat = log_state(:, 1:4);
des_traj = readmatrix('../input/motion_planner_interface_06_26_20/DATA/Results.dat');

addpath('./plotting');
anim_tumble(des_traj(:,2:4), R0_mat, des_traj(:,2:4));