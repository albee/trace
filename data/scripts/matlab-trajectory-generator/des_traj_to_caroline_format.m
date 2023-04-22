%{
Exports a reference trajectory to follow for 6 DOF RBD dynamics.

Input:
WRITE_PATH: where the output goes

des_traj: [
  t x y z xd yd zd qw qx qy qz wx wy wz xdd ydd zdd wxd wyd wzd
  ...
]

u_des: [
  t ux uy uz
  ...
]

name: directory name

Output Files:
  results_pos_0.dat, results_vel_0.dat, results_ori_0.dat, results_ome_0.dat
  results_input_0.dat
%}
function des_traj_to_caroline_format(WRITE_PATH, des_traj, u_des, name)
    t = des_traj(:,1);
    results_pos = [t des_traj(:,2:4)];
    results_vel = [t des_traj(:,5:7)];
    results_acc = [t des_traj(:,15:17)];
    
    results_ori = [t des_traj(:,8:11)];
    results_ome = [t des_traj(:,12:14)];
    
    results_input = [t u_des];
    
    PATH = strcat(WRITE_PATH, name);
    mkdir(PATH)

    % generate .dat files
    % output_x, translation
    fid = fopen(strcat(PATH, '/results_pos_0.dat'),'wt');
    for i = 1:length(t)
        fprintf(fid, '%14.4f%14.4f%14.4f%14.4f\n', results_pos(i,1), results_pos(i,2), results_pos(i,3), results_pos(i,4));
    end
    fclose(fid);
    
    fid = fopen(strcat(PATH, '/results_vel_0.dat'),'wt');
    for i = 1:length(t)
        fprintf(fid, '%14.4f%14.4f%14.4f%14.4f\n', results_vel(i,1), results_vel(i,2), results_vel(i,3), results_vel(i,4));
    end
    fclose(fid);
    
    fid = fopen(strcat(PATH, '/results_acc_0.dat'),'wt');
    for i = 1:length(t)
        fprintf(fid, '%14.4f%14.4f%14.4f%14.4f\n', results_acc(i,1), results_acc(i,2), results_acc(i,3), results_acc(i,4));
    end
    fclose(fid);
    
    % output_x, rotation
    fid = fopen(strcat(PATH, '/results_ori_0.dat'),'wt');
    for i = 1:length(t)
        fprintf(fid, '%14.4f%14.4f%14.4f%14.4f%14.4f\n', results_ori(i,1), results_ori(i,2), results_ori(i,3), results_ori(i,4), results_ori(i, 5));
    end
    fclose(fid);
    
    fid = fopen(strcat(PATH, '/results_ome_0.dat'),'wt');
    for i = 1:length(t)
        fprintf(fid, '%14.4f%14.4f%14.4f%14.4f\n', results_ome(i,1), results_ome(i,2), results_ome(i,3), results_ome(i,4));
    end
    fclose(fid);
        
    %output_u
    fid = fopen(strcat(PATH, '/results_input_0.dat'),'wt');
    for i = 1:length(t)
        fprintf(fid, '%14.4f%14.4f%14.4f%14.4f\n', results_input(i,1), results_input(i,2), results_input(i,3), results_input(i,4));
    end
    fclose(fid);
end
