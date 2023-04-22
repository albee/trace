%{
Exports a reference trajectory to follow for 6 DOF dynamics, 
Currently produces up to 2nd derivative for double integrator systems.

des_traj of format: [t x1 x2 x3 x1d x2d x3d x1dd x2dd x3dd];
%}
function des_traj_to_roberto_format(des_traj)
    t = des_traj(:,1);
    results = [t des_traj(:,2:4)];
    results_vel = [t des_traj(:,5:7)];
    results_acc = [t des_traj(:,8:10)];
    results_jerk = [t zeros(length(t),3)];
    
    mkdir('../input/my-sample-traj')

    % generate csvs
    fid = fopen('DATA_DLR_FORMAT/Results.dat','wt');
    for i = 1:length(t)
        fprintf(fid, '%14.4f%14.4f%14.4f%14.4f\n', results(i,1), results(i,2), results(i,3), results(i,4));
    end
    fclose(fid);
    
    fid = fopen('DATA_DLR_FORMAT/Results_vel.dat','wt');
    for i = 1:length(t)
        fprintf(fid, '%14.4f%14.4f%14.4f%14.4f\n', results_vel(i,1), results_vel(i,2), results_vel(i,3), results_vel(i,4));
    end
    fclose(fid);
    
    fid = fopen('DATA_DLR_FORMAT/Results_acc.dat','wt');
    for i = 1:length(t)
        fprintf(fid, '%14.4f%14.4f%14.4f%14.4f\n', results_acc(i,1), results_acc(i,2), results_acc(i,3), results_acc(i,4));
    end
    fclose(fid);
    
    fid = fopen('DATA_DLR_FORMAT/Results_jerk.dat','wt');
    for i = 1:length(t)
        fprintf(fid, '%14.4f%14.4f%14.4f%14.4f\n', results_jerk(i,1), results_jerk(i,2), results_jerk(i,3), results_jerk(i,4));
    end
    fclose(fid);
end
