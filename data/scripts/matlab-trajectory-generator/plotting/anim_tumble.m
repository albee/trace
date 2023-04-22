%{
 Takes in a matrix of r0 and a matrix of q and animates. Time varies with the
 row.

Input
r0_mat - matrix of [x, y, z] position
R0_mat - matrix of [qx, qy, qz, qw] quaternions of orientation. Scalar LAST, represents B wrt I
des_traj - in INERTIAL frame [x, y, z]. Plots separately.
%}
function anim_tumble(r0_mat, R0_mat, des_traj)
  fig = figure('units','normalized','outerposition',[0 0 1 1]);
  set(gca,'zdir','reverse', 'xdir','reverse')
  grid on
%   view(180, 0);
  view(145, 45);
  hold on
  axis equal;
  start_pos = r0_mat(1, :);
  axis([-2, 2, -3, 3, -2, 2] + [start_pos(1), start_pos(1), start_pos(2), start_pos(2), start_pos(3), start_pos(3)]);
  xlabel('x [m]')
  ylabel('y [m]')
  zlabel('z [m]')
  x = des_traj(:,1);
  y = des_traj(:,2);
  z = des_traj(:,3);
  plot3(x, y, z, 'black', 'Linewidth', 3);
  pause(1.0)

  anim_FK3(r0_mat, R0_mat, fig);
end