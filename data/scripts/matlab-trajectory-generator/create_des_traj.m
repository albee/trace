%{
Create a reference trajectory to follow using RBD dynamics.
Currently produces up to 2nd derivative.

Traj length is tf/dt + 1, from 0 to tf.

Input:
dt - timestep
tf - final time
DIMS - system name
DES_TRAJ - des traj name
START_POS
START_ATT

Output:
des_traj: [
  t x y z xd yd zd qx qy qz qw wx wy wz xdd ydd zdd wxd wyd wzd
  ...
]

u_des: [
  t ux uy uz
  ...
]
%}

function [des_traj, u_des] = create_des_traj(dt, tf, DES_TRAJ, START_POS, START_ATT)
    START_POS = START_POS';
    num_setpoints = ceil(tf/dt) + 1;  % same as N---inclusive of start and end [t0; ...; tN]
    t_des_hist = linspace(0,tf,num_setpoints)';
    
    % moments of inertia (principal coordinates)
    I_xx = 0.153;
    I_yy = 0.143;
    I_zz = 0.162;

    % inertia ratios
    p_x = (I_yy - I_zz)/I_xx;
    p_y = (I_zz - I_xx)/I_yy;
    p_z = (I_xx - I_yy)/I_zz;

    switch DES_TRAJ
        case 'SINUSOID'
            samples1 = linspace(0, 2*pi*1, num_setpoints);
            x = [0.05*sin(samples1)', 0.03*sin(samples1)', 0.02*sin(samples1)'];  % positions to follow

            x_des_hist = [x];
            xd_des_hist = [[0; diff(x_des_hist(:,1))] [0; diff(x_des_hist(:,2))] [0; diff(x_des_hist(:,3))]]/dt;
            xdd_des_hist = [[0; diff(xd_des_hist(:,1))] [0; diff(xd_des_hist(:,2))] [0; diff(xd_des_hist(:,3))]]/dt;  % accelerations
            
            q_des_hist = zeros(num_setpoints, 4);
            q_des_hist(:, 4) = 1;
            w_des_hist = zeros(num_setpoints, 3);
            wd_des_hist = zeros(num_setpoints, 3);

        case 'STEPX'
            x_des_hist = [[zeros(ceil(num_setpoints*.1),1); ones(floor(num_setpoints*.9),1)] zeros(ceil(num_setpoints), 2)];
            xd_des_hist = [[0; diff(x_des_hist(:,1))] [0; diff(x_des_hist(:,2))] [0; diff(x_des_hist(:,3))]]/dt;
            xdd_des_hist = [[0; diff(xd_des_hist(:,1))] [0; diff(xd_des_hist(:,2))] [0; diff(xd_des_hist(:,3))]]/dt;  % accelerations
            
            q_des_hist = zeros(num_setpoints, 4);
            q_des_hist(:, 4) = 1;
            w_des_hist = zeros(num_setpoints, 3);
            wd_des_hist = zeros(num_setpoints, 3);

       case 'STEPY'
            x_des_hist = [zeros(ceil(num_setpoints), 1) [zeros(ceil(num_setpoints*.1),1); ones(floor(num_setpoints*.9),1)] zeros(ceil(num_setpoints), 1)];
            xd_des_hist = [[0; diff(x_des_hist(:,1))] [0; diff(x_des_hist(:,2))] [0; diff(x_des_hist(:,3))]]/dt;
            xdd_des_hist = [[0; diff(xd_des_hist(:,1))] [0; diff(xd_des_hist(:,2))] [0; diff(xd_des_hist(:,3))]]/dt;  % accelerations
            
            q_des_hist = zeros(num_setpoints, 4);
            q_des_hist(:, 4) = 1;
            w_des_hist = zeros(num_setpoints, 3);
            wd_des_hist = zeros(num_setpoints, 3);

      case 'STEPZ'
            x_des_hist = [zeros(ceil(num_setpoints), 2 ) [zeros(ceil(num_setpoints*.1),1); ones(floor(num_setpoints*.9),1)]];
            xd_des_hist = [[0; diff(x_des_hist(:,1))] [0; diff(x_des_hist(:,2))] [0; diff(x_des_hist(:,3))]]/dt;
            xdd_des_hist = [[0; diff(xd_des_hist(:,1))] [0; diff(xd_des_hist(:,2))] [0; diff(xd_des_hist(:,3))]]/dt;  % accelerations
            
            q_des_hist = zeros(num_setpoints, 4);
            q_des_hist(:, 4) = 1;
            w_des_hist = zeros(num_setpoints, 3);
            wd_des_hist = zeros(num_setpoints, 3);

      case 'ZERO'

            x_des_hist = zeros(ceil(num_setpoints), 3 );
            xd_des_hist = [[0; diff(x_des_hist(:,1))] [0; diff(x_des_hist(:,2))] [0; diff(x_des_hist(:,3))]]/dt;
            xdd_des_hist = [[0; diff(xd_des_hist(:,1))] [0; diff(xd_des_hist(:,2))] [0; diff(xd_des_hist(:,3))]]/dt;  % accelerations
            
            q_des_hist = zeros(num_setpoints, 4);
            q_des_hist(:, 4) = 1;
            w_des_hist = zeros(num_setpoints, 3);
            wd_des_hist = zeros(num_setpoints, 3);

      case 'ATTITUDE_X_STEPS'
            x_des_hist = zeros(ceil(num_setpoints), 3);
            xd_des_hist = zeros(ceil(num_setpoints), 3);
            xdd_des_hist = zeros(ceil(num_setpoints), 3);
            
            tumble_dynamics = casadi_6dof_tumble_dynamics(0);  % create the RBD attitude dynamics function tumble_dynamics( )

            u_0 = [0.1, 0.0, 0.0];
            x_i = [START_ATT; 0.0; 0; 0];
            log_ang_state = [];
            for i = 0:dt:tf
              log_ang_state(end+1, :) = x_i';
              
              if i < 2.3*tf/10
                u_i = u_0;
              elseif i < 7.0*tf/10
                u_i = -u_0;
              else
                u_i = u_0;
              end

              x_i = full(tumble_dynamics(x_i, u_i, [p_x; p_y; p_z], dt));  % full is required to convert from casadi.DM    
                % x = [qx; qy; qz; qw; wx; wy; wz], [-] B wrt I, [rad/s] B frame
                % u = [u1; u2; u3], [N]
                % p = [p_x; p_y; p_z]  non-dimensional inertia ratios
            end
            
            q_des_hist = log_ang_state(:,1:4);
            w_des_hist = log_ang_state(:, 5:7);
            wd_des_hist = zeros(num_setpoints, 3);
            
         case 'ATTITUDE_Y_STEPS'
            x_des_hist = zeros(ceil(num_setpoints), 3);
            xd_des_hist = zeros(ceil(num_setpoints), 3);
            xdd_des_hist = zeros(ceil(num_setpoints), 3);
            
            tumble_dynamics = casadi_6dof_tumble_dynamics(0);  % create the RBD attitude dynamics function tumble_dynamics( )

            u_0 = [0.0, 0.04, 0.0];
            x_i = [START_ATT; 0.0; 0; 0];
            log_ang_state = [];
            for i = 0:dt:tf
              log_ang_state(end+1, :) = x_i';
              
              if i < 2.3*tf/10
                u_i = u_0;
              elseif i < 7.0*tf/10
                u_i = -u_0;
              else
                u_i = u_0;
              end

              x_i = full(tumble_dynamics(x_i, u_i, [p_x; p_y; p_z], dt));  % full is required to convert from casadi.DM    
                % x = [qx; qy; qz; qw; wx; wy; wz], [-] B wrt I, [rad/s] B frame
                % u = [u1; u2; u3], [N]
                % p = [p_x; p_y; p_z]  non-dimensional inertia ratios
            end
            
            q_des_hist = log_ang_state(:,1:4);
            w_des_hist = log_ang_state(:, 5:7);
            wd_des_hist = zeros(num_setpoints, 3);
            
         case 'ATTITUDE_Z_STEPS'
            x_des_hist = zeros(ceil(num_setpoints), 3);
            xd_des_hist = zeros(ceil(num_setpoints), 3);
            xdd_des_hist = zeros(ceil(num_setpoints), 3);
            
            tumble_dynamics = casadi_6dof_tumble_dynamics(0);  % create the RBD attitude dynamics function tumble_dynamics( )

            u_0 = [0.0, 0.0, 0.01];
            x_i = [START_ATT; 0.0; 0; 0];
            log_ang_state = [];
            for i = 0:dt:tf
              log_ang_state(end+1, :) = x_i';
              
               if i < 2.3*tf/10
                u_i = u_0;
              elseif i < 7.0*tf/10
                u_i = -u_0;
              else
                u_i = u_0;
              end

              x_i = full(tumble_dynamics(x_i, u_i, [p_x; p_y; p_z], dt));  % full is required to convert from casadi.DM    
                % x = [qx; qy; qz; qw; wx; wy; wz], [-] B wrt I, [rad/s] B frame
                % u = [u1; u2; u3], [N]
                % p = [p_x; p_y; p_z]  non-dimensional inertia ratios
            end
            
            q_des_hist = log_ang_state(:,1:4);
            w_des_hist = log_ang_state(:, 5:7);
            wd_des_hist = zeros(num_setpoints, 3);
            
         case 'ATTITUDE_STEPS'
            x_des_hist = zeros(ceil(num_setpoints), 3);
            xd_des_hist = zeros(ceil(num_setpoints), 3);
            xdd_des_hist = zeros(ceil(num_setpoints), 3);
            
            tumble_dynamics = casadi_6dof_tumble_dynamics(0);  % create the RBD attitude dynamics function tumble_dynamics( )

            u_0 = [0.1, 0.1, 0.1];
            x_i = [START_ATT; 0.0; 0; 0];
            log_ang_state = [];
            for i = 0:dt:tf
              log_ang_state(end+1, :) = x_i';
              
               if i < 2.3*tf/10
                u_i = u_0;
              elseif i < 7.0*tf/10
                u_i = -u_0;
              else
                u_i = u_0;
              end

              x_i = full(tumble_dynamics(x_i, u_i, [p_x; p_y; p_z], dt));  % full is required to convert from casadi.DM    
                % x = [qx; qy; qz; qw; wx; wy; wz], [-] B wrt I, [rad/s] B frame
                % u = [u1; u2; u3], [N]
                % p = [p_x; p_y; p_z]  non-dimensional inertia ratios
            end
            
            q_des_hist = log_ang_state(:,1:4);
            w_des_hist = log_ang_state(:, 5:7);
            wd_des_hist = zeros(num_setpoints, 3);
          case 'TEST3-GND'
            x_des_hist = [zeros(ceil(num_setpoints*.1), 3); ...
              [-ones(ceil(num_setpoints*.45), 1)*0.20, zeros(ceil(num_setpoints*.45), 2)]; ...
              [-ones(floor(num_setpoints*.45), 2)*0.20, zeros(floor(num_setpoints*.45), 1)]];
            xd_des_hist = [[0; diff(x_des_hist(:,1))] [0; diff(x_des_hist(:,2))] [0; diff(x_des_hist(:,3))]]/dt;
            xdd_des_hist = [[0; diff(xd_des_hist(:,1))] [0; diff(xd_des_hist(:,2))] [0; diff(xd_des_hist(:,3))]]/dt;  % accelerations
            
            q_des_hist = zeros(num_setpoints, 4);
            q_des_hist(:, 3) = -0.7071068;
            q_des_hist(:, 4) = 0.7071068;
            w_des_hist = zeros(num_setpoints, 3);
            wd_des_hist = zeros(num_setpoints, 3);
          case 'TEST3-ISS'
            x_des_hist = [zeros(ceil(num_setpoints*.1), 3); ...
                          [ones(floor(num_setpoints*.3), 1)*0.40, zeros(floor(num_setpoints*.3), 2)]; ...
                          [ones(floor(num_setpoints*.3), 2)*0.40, zeros(floor(num_setpoints*.3), 1)]; ...
                          [ones(floor(num_setpoints*.3), 3)*0.40]];
            xd_des_hist = [[0; diff(x_des_hist(:,1))] [0; diff(x_des_hist(:,2))] [0; diff(x_des_hist(:,3))]]/dt;
            xdd_des_hist = [[0; diff(xd_des_hist(:,1))] [0; diff(xd_des_hist(:,2))] [0; diff(xd_des_hist(:,3))]]/dt;  % accelerations
            
            q_des_hist = zeros(num_setpoints, 4);
            q_des_hist(:, 3) = START_ATT(3);
            q_des_hist(:, 4) = START_ATT(4);
            w_des_hist = zeros(num_setpoints, 3);
            wd_des_hist = zeros(num_setpoints, 3);

          case 'MIT-ISS'
            waypoints = [
            0, 0, 0;
            0, 0, -0.4;
            0, -0.2, 0.0;
            0, -0.4, -0.4;
            0, -0.4, 0.0;  % M
            0, -0.6, 0.0;
            0, -0.6, -0.4;
            0, -0.4, -0.4;
            0, -0.8, -0.4;
            0, -0.6, -0.4;
            0, -0.6, -0.0;  % I
            0, -1.0, -0.0;
            0, -1.0, -0.4;
            0, -0.8, -0.4;
            0, -1.2, -0.4;  % T
            ];
            x_des_hist = [];
            for i=1:1:(length(waypoints) - 1)
                x_new = linspace_3d(waypoints(i, :), waypoints(i+1, :));
                x_des_hist = [x_des_hist; x_new];
            end

            xd_des_hist = [[0; diff(x_des_hist(:,1))] [0; diff(x_des_hist(:,2))] [0; diff(x_des_hist(:,3))]]/dt;
            xdd_des_hist = [[0; diff(xd_des_hist(:,1))] [0; diff(xd_des_hist(:,2))] [0; diff(xd_des_hist(:,3))]]/dt;  % accelerations
            

            q_des_hist = zeros(length(x_des_hist), 4);
            q_des_hist(:, 3) = START_ATT(3);
            q_des_hist(:, 4) = START_ATT(4);
            w_des_hist = zeros(length(x_des_hist), 3);
            wd_des_hist = zeros(length(x_des_hist), 3);

            t_des_hist = 0.0:dt:(length(x_des_hist) - 1)*dt;
            t_des_hist = t_des_hist';
size(t_des_hist)

            num_setpoints = length(x_des_hist);

      otherwise
        disp('Trajectory not defined!');
        return
    end

    %{
    des_traj: [
      t x y z xd yd zd qw qx qy qz wx wy wz xdd ydd zdd wxd wyd wzd
      ...
    ]
    %}
    des_traj = [t_des_hist, x_des_hist, xd_des_hist, q_des_hist, w_des_hist, xdd_des_hist, wd_des_hist];
    des_traj = shift_start_pos(START_POS, des_traj);
    
    %{
    output_u: [
      t ux uy uz
      ...
    ]
    %}
    u_des = zeros(num_setpoints, 3);  % TODO
end

function lin = linspace_3d(v1, v2)
    n = 50;
    t = linspace(0,1,n)';
    lin = (1-t)*v1 + t*v2;
end

function des_traj = shift_start_pos(START_POS, des_traj)
  des_traj(:, 2:4) = des_traj(:, 2:4) + START_POS;
end