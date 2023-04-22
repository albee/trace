%{
Keenan Albee, Oct-15 2020
Tumbling target dynamcis using CasADi.
See `propagate_tumble_dynamics` for an example.


Inputs:


Outputs:
tumble_dynamics(x, u, p, dt) - CasADi function

%}

function [tumble_dynamics] = casadi_6dof_tumble_dynamics(GEN_CODE)
  import casadi.*

  %% Set up symbolic variables
  % Continuous-time dynamics!
  % x = [qx; qy; qz; qw; wx; wy; wz], [-] B wrt I, [rad/s] B frame
  % u = [u1; u2; u3], [N]
  % p = [p_x; p_y; p_z]  non-dimensional inertia ratios

  x = MX.sym('x', 7, 1); % States   
  u = MX.sym('u', 3, 1); % Controls
  p = MX.sym('p', 3, 1); % Parameters
  tf = MX.sym('tf');  % final time of integration, determined by time horizons
  p_stacked = [u; p; tf];

  %% Set up ODE
  % See [Albee 2019], [Aghili 2008]
  % quaternion dynamics
  qx = x(1);
  qy = x(2);
  qz = x(3);
  qw = x(4);
  wx = x(5);
  wy = x(6);
  wz = x(7);
  px = p(1);
  py = p(2);
  pz = p(3);
  
  w = [wx; wy; wz];
  H = [ qw,  qz, -qy, -qx;
       -qz,  qw,  qx, -qy;
        qy, -qx,  qw, -qz]; % assumes scalar LAST! quaternion is B wrt I
  qd = 1/2*H'*w;

  % tumble angular velocity dynamics
  phi = [px*wy*wz;
         py*wx*wz;
         pz*wx*wy];
       
  B = [1, 0, 0;
       0, (1+py)/(1+px), 0;
       0, 0, (1+pz)/(1-px)];
     
  wd = phi + B*u;

  xdot = [qd; wd];

  f = Function('f', {x, u, p}, {xdot}, {'x','u','p'}, {'ode'});
  f([1; 0; 0; 0; 1; 1; 1], [0; 0; 0], [0.5; 0.5; 0.5]); % {x, u}

  
  %% Create an integrator
  % Integrator to discretize the system
  intg_options = struct;
  intg_options.tf = 1.0;  % final time, modified by input time
  intg_options.simplify = true;
  intg_options.number_of_finite_elements = 4;

  % DAE problem structure
  dae = struct;
  dae.x = x;         % What are states?
  dae.p = p_stacked;    % What are parameters (=fixed during the integration horizon)?
  dae.ode = f(x, u, p)*tf;  % Expression for the right-hand side

  intg = integrator('intg', 'rk', dae, intg_options);

  % x0 = [0;1];
  % res = intg('x0',[0;1],'p',0);  % Easier to identify inputs, but bloated API
  % res.xf;  % view final state

  % Simplify API to (x,u)->(x_next)
  res = intg('x0', x, 'p', p_stacked); % Evaluate with symbols
  x_next = res.xf;  % symbolically defined
  tumble_dynamics = Function('tumble_dynamics', {x, u, p, tf}, {x_next},{'x', 'u', 'p', 'tf'},{'x_next'});  % function call to integrate: continuous over timestep dt

  % sim = integrator_func_casadi.mapaccum(N);  % take x0 and |u|xN matrix, output N states propagated forward

  %% Check angular velocity magnitude is unchanged
  % moments of inertia
  I_xx = 2.0;
  I_yy = 2.0;
  I_zz = 1.0;

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

  % angular velocity
  w_x0 = 1.0;
  w_y0 = 1.0;
  w_z0 = 1.0;
  w_i = [w_x0; w_y0; w_z0];
  w1 = full(norm([w_x0, w_y0, w_z0]));

  x_i = [q_i; w_i];

  t_next = 1.0;

  x_next = tumble_dynamics(x_i, [0; 0; 0], [p_x; p_y; p_z], t_next);
  w2 = full(norm(x_next(5:7)));

  assert((w1 - w2) < 0.001, 'Angular velocity is not conserved!')

  %% Code gen with embedded SQP solver
  if GEN_CODE == 1
  %     mpc_func_casadi.generate('casadi_3dof_double_integrator', struct('mex',true, 'with_header', true));
      CodeGen = CodeGenerator('casadi_3dof_double_integrator.c', struct('with_header', true));
      CodeGen.add(mpc_func_casadi)
      CodeGen.add(integrator_func_casadi)

      disp('Generating C code...')
      CodeGen.generate();
      % mex casadi_3dof_double_integrator.c -DMATLAB_MEX_FILE
      disp('...done')

  %     format long
  %     codegen_demo('mpc_func_casadi',x0)
  end
end

