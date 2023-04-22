function [x, v, quat, omega] = read_in_caroline_format(PATH)
  %{
  Read in a tumble trajectory using Caroline's output format.

  Input:
  PATH - path to trajectory *.dat folder.
  %}

  % des traj, in inertial coordinates
  x = readmatrix(strcat(PATH, '/results_pos_0.dat'));
  v = readmatrix(strcat(PATH, '/results_vel_0.dat'));
  quat = readmatrix(strcat(PATH, '/results_ori_0.dat'));
  omega = readmatrix(strcat(PATH, '/results_ome_0.dat'));
  
  x = x(:, 2:4);
  v = v(:, 2:4);
  quat = quat(:, 2:5);
  omega = omega(:, 2:4);
end