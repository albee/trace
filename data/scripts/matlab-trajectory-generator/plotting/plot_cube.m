function handles = plot_cube(r0, R0, l, clr, alpha)
%{
plot_cube - Display a 3D-cube in the current axes, with translation and
rotation. Adapted by Keenan Albee from Olivier's PLOTCUBE function.

  plot_cube(r0, R0, l, clr, alpha) displays a 3D-cube in the current axes
  with the following properties:
  * r0 : 3x1 vector that defines the cube origin, I_r0_IB
  * R0: 3x3 rotation matrix that defines cube orientation: body relative
      to inertial, R0_I_B
  * l : cube side length
  * clr : 3-element vector that defines the faces color of the cube
  * alpha : scalar of opacity

  * handles: handles to everything that got drawn
%}
  l2 = l/2;

  XYZ = { ... % faces, each has 4 points
    [0 0 0 0]  [0 0 1 1]  [0 1 1 0] ; ... 
    [1 1 1 1]  [0 0 1 1]  [0 1 1 0] ; ...
    [0 1 1 0]  [0 0 0 0]  [0 0 1 1] ; ...
    [0 1 1 0]  [1 1 1 1]  [0 0 1 1] ; ...
    [0 1 1 0]  [0 0 1 1]  [0 0 0 0] ; ...
    [0 1 1 0]  [0 0 1 1]  [1 1 1 1]   ...
    };

  for i=1:6 % ith face
      x = XYZ{i,1}; y = XYZ{i,2}; z = XYZ{i,3};
      % adjust size
      pt1 = [x(1), y(1), z(1)]'*l - l2;
      pt2 = [x(2), y(2), z(2)]'*l - l2;
      pt3 = [x(3), y(3), z(3)]'*l - l2;
      pt4 = [x(4), y(4), z(4)]'*l - l2;

      % convert b_r0 to i_r0
      pt1 = R0*pt1 + r0;
      pt2 = R0*pt2 + r0;
      pt3 = R0*pt3 + r0;
      pt4 = R0*pt4 + r0;

      % reassemble for patch
      XYZ{i,1} = [pt1(1) pt2(1) pt3(1) pt4(1)];
      XYZ{i,2} = [pt1(2) pt2(2) pt3(2) pt4(2)];
      XYZ{i,3} = [pt1(3) pt2(3) pt3(3) pt4(3)];
  end

  % Set triad face colors
  color_mat = repmat({clr},6,1);
  color_mat{2} = [1,0,0];
  color_mat{4} = [0,1,0];
  color_mat{3} = [0,0,1];

  h1 = cellfun(@patch,XYZ(:,1),XYZ(:,2),XYZ(:,3),...
    color_mat,...
    repmat({'FaceAlpha'},6,1),...
    repmat({alpha},6,1)...
    );

  %% Optional: Rotate and plot triad
  pt0 = [0;0;0];
  pt1 = [1.5*l;0;0];
  pt2 = [0;1.5*l;0];
  pt3 = [0;0;1.5*l];

  pt0 = pt0 + r0;
  pt1 = R0*pt1;
  pt2 = R0*pt2;
  pt3 = R0*pt3;

  h2 = quiver3(pt0(1), pt0(2), pt0(3), pt1(1), pt1(2), pt1(3));
  h2.Color = 'r';
  h2.LineWidth = 2.0;

  h3 = quiver3(pt0(1), pt0(2), pt0(3), pt2(1), pt2(2), pt2(3));
  h3.Color = 'g';
  h3.LineWidth = 2.0;

  h4 = quiver3(pt0(1), pt0(2), pt0(3), pt3(1), pt3(2), pt3(3));
  h4.Color = 'b';
  h4.LineWidth = 2.0;

  handles = [h1; h2; h3; h4];
end

