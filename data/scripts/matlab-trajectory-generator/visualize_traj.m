%% Reads in a directory of Caroline output format
close all;
PATH = '../../../motion_planner_interface/bin/iss-traj1-public';
% PATH = strcat('../input/', REF_TRAJ);
[x, v, quat, omega] = read_in_caroline_format(PATH);

%% Visualizes a Caroline output format
addpath('./plotting');
%anim_tumble(x, quat, x);

%% Plot trajectory

t = 0:0.2:60;
figure;
plot(t, x(:,1), 'r-');
hold on;
plot(t, x(:,2), 'g-');
plot(t, x(:,3), 'b-');
grid on;
ylabel('(m)');
xlabel('(s)');

figure;
plot(t, v(:,1), 'r-');
hold on;
plot(t, v(:,2), 'g-');
plot(t, v(:,3), 'b-');
grid on;
ylabel('(m/s)');
xlabel('(s)');

figure;
plot(t, quat(:,1), 'r-');
hold on;
plot(t, quat(:,2), 'g-');
plot(t, quat(:,3), 'b-');
plot(t, quat(:,4), 'k-');
grid on;
ylabel('quat');
xlabel('(s)');

figure;
plot(t, omega(:,1), 'r-');
hold on;
plot(t, omega(:,2), 'g-');
plot(t, omega(:,3), 'b-');
grid on;
ylabel('(rad/s)');
xlabel('(s)');

norm_pos = zeros(1,length(x(:,1)));
for i = 1:length(x(:,1))
    norm_pos(i) = norm(x(i,:));
end