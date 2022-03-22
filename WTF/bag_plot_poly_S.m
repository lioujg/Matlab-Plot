% Plot
% read experiment data from bag file and plot the trajectory
close all

% parameters for plotting
bag_select = "2021-08-19-15-22-37.bag";

if      bag_select == "2021-08-19-15-20-47.bag"
        length_of_t = 16184;
        pos_front_null = 1;
        pos_back_null = 970;
        t_front_null = pos_front_null*2;
        t_back_null = pos_back_null*2;
        desired_pos_front_null = 216;
        desired_pos_back_null = 531;
        c2_desired_Z = ones(desired_pos_back_null-desired_pos_front_null-1 ,1) * 0.36;
elseif  bag_select == "2021-08-19-15-22-37.bag"
        length_of_t = 16184;
        pos_front_null = 1;
        pos_back_null = 970;
        t_front_null = pos_front_null*2;
        t_back_null = pos_back_null*2;
        desired_pos_front_null = 207;
        desired_pos_back_null = 522;
        c2_desired_Z = ones(desired_pos_back_null-desired_pos_front_null-1 ,1) * 0.31;
elseif  bag_select == "2021-08-19-15-24-21.bag"
        length_of_t = 16184;
        pos_front_null = 1;
        pos_back_null = 970;
        t_front_null = pos_front_null*2;
        t_back_null = pos_back_null*2;
        desired_pos_front_null = 209;
        desired_pos_back_null = 524;
        c2_desired_Z = ones(desired_pos_back_null-desired_pos_front_null-1 ,1) * 0.31;
elseif  bag_select == "2021-08-19-15-26-38.bag"
        length_of_t = 16184;
        pos_front_null = 1;
        pos_back_null = 970;
        t_front_null = pos_front_null*2;
        t_back_null = pos_back_null*2;
        desired_pos_front_null = 249;
        desired_pos_back_null = 562;
        c2_desired_Z = ones(desired_pos_back_null-desired_pos_front_null-1 ,1) * 0.31;
elseif  bag_select == "2021-08-19-15-32-21.bag"
        length_of_t = 16184;
        pos_front_null = 1;
        pos_back_null = 970;
        t_front_null = pos_front_null*2;
        t_back_null = pos_back_null*2;
        desired_pos_front_null = 349;
        desired_pos_back_null = 664;
        c2_desired_Z = ones(desired_pos_back_null-desired_pos_front_null-1 ,1) * 0.31;
elseif  bag_select == "2021-08-19-15-39-48.bag"
        length_of_t = 16184;
        pos_front_null = 1;
        pos_back_null = 970;
        t_front_null = pos_front_null*2;
        t_back_null = pos_back_null*2;
        desired_pos_front_null = 275;
        desired_pos_back_null = 590;
        c2_desired_Z = ones(desired_pos_back_null-desired_pos_front_null-1 ,1) * 0.31;
end

% read data from bag file
bag = rosbag(bag_select);

% time
time_start = bag.StartTime;
time_end = bag.EndTime;
time_duration = time_end - time_start;

% obtain the clock
t_bag = select(bag, 'topic', 'rosout');
t_msgStructs = readMessages(t_bag, 'DataFormat', 'struct');
t = cellfun(@(m) double(m.Header.Stamp.Sec), t_msgStructs);
t_nsec = cellfun(@(m) double(m.Header.Stamp.Nsec), t_msgStructs);
t = t - t(1);
t = t + t_nsec*10^(-9);

% obtain MAV1 pose
MAV1_bag = select(bag, 'topic', 'vrpn_client_node/MAV1/pose');
MAV1_msgStructs = readMessages(MAV1_bag, 'DataFormat', 'struct');
MAV1_pose_X = cellfun(@(m) double(m.Pose.Position.X), MAV1_msgStructs);
MAV1_pose_Y = cellfun(@(m) double(m.Pose.Position.Y), MAV1_msgStructs);
MAV1_pose_Z = cellfun(@(m) double(m.Pose.Position.Z), MAV1_msgStructs);

% MAV2 pose
MAV2_bag = select(bag, 'topic', 'vrpn_client_node/MAV2/pose');
MAV2_msgStructs = readMessages(MAV2_bag, 'DataFormat', 'struct');
MAV2_pose_X = cellfun(@(m) double(m.Pose.Position.X), MAV2_msgStructs);
MAV2_pose_Y = cellfun(@(m) double(m.Pose.Position.Y), MAV2_msgStructs);
MAV2_pose_Z = cellfun(@(m) double(m.Pose.Position.Z), MAV2_msgStructs);

% c2 point
c2_bag = select(bag, 'topic', 'pc2_debug');
c2_msgStructs = readMessages(c2_bag, 'DataFormat', 'struct');
c2_X = cellfun(@(m) double(m.X), c2_msgStructs);
c2_Y = cellfun(@(m) double(m.Y), c2_msgStructs);
c2_Z = cellfun(@(m) double(m.Z), c2_msgStructs);

% desired c2 point
c2_desired_bag = select(bag, 'topic', 'path_plot');
c2_desired_msgStructs = readMessages(c2_desired_bag, 'DataFormat', 'struct');
c2_desired_X = cellfun(@(m) double(m.X), c2_desired_msgStructs);
c2_desired_Y = cellfun(@(m) double(m.Y), c2_desired_msgStructs);
%c2_desired_Z = cellfun(@(m) double(m.Z), c2_desired_msgStructs);

% figure 12 eta1
v_hat_bag = select(bag, 'topic', 'v_hat_plot_f12');
v_hat_msgStructs = readMessages(v_hat_bag, 'DataFormat', 'struct');
v_hat = cellfun(@(m) double(m.X), v_hat_msgStructs);

vd_bag = select(bag, 'topic', 'vd_plot_f12');
vd_msgStructs = readMessages(vd_bag, 'DataFormat', 'struct');
vd = cellfun(@(m) double(m.X), vd_msgStructs);

eta1_bag = select(bag, 'topic', 'eta1_plot_f12');
eta1_msgStructs = readMessages(eta1_bag, 'DataFormat', 'struct');
eta1 = cellfun(@(m) double(m.X), eta1_msgStructs);

% figure 13 eta2
omega_bag = select(bag, 'topic', 'omega_plot_f13');
omega_msgStructs = readMessages(omega_bag, 'DataFormat', 'struct');
omega = cellfun(@(m) double(m.Z),omega_msgStructs);

omega_d_bag = select(bag, 'topic', 'omega_d_plot_f13');
omega_d_msgStructs = readMessages(omega_d_bag, 'DataFormat', 'struct');
omega_d = cellfun(@(m) double(m.X), omega_d_msgStructs);

eta2_bag = select(bag, 'topic', 'eta2_plot_f13');
eta2_msgStructs = readMessages(eta2_bag, 'DataFormat', 'struct');
eta2 = cellfun(@(m) double(m.X), eta2_msgStructs);

% figure 14, 15 FL
FL_bag = select(bag, 'topic', 'FL_plot_f14_15');
FLx_msgStructs = readMessages(FL_bag, 'DataFormat', 'struct');
FLx = cellfun(@(m) double(m.X),FLx_msgStructs);

FLy_msgStructs = readMessages(FL_bag, 'DataFormat', 'struct');
FLy = cellfun(@(m) double(m.Y),FLy_msgStructs);

% delete useless points
MAV1_pose_X(pos_back_null:end) = [];
MAV1_pose_Y(pos_back_null:end) = [];
MAV1_pose_Z(pos_back_null:end) = [];
MAV2_pose_X(pos_back_null:end) = [];
MAV2_pose_Y(pos_back_null:end) = [];
MAV2_pose_Z(pos_back_null:end) = [];
c2_X(pos_back_null:end) = [];
c2_Y(pos_back_null:end) = [];
c2_Z(pos_back_null:end) = [];
c2_desired_X(desired_pos_back_null:end) = [];
c2_desired_Y(desired_pos_back_null:end) = [];
%c2_desired_Z(desired_pos_back_null:end) = [];
v_hat(desired_pos_back_null:end) = [];
vd(desired_pos_back_null:end) = [];
eta1(desired_pos_back_null:end) = [];
omega(desired_pos_back_null:end) = [];
omega_d(desired_pos_back_null:end) = [];
eta2(desired_pos_back_null:end) = [];
FLx(desired_pos_back_null:end) = [];
FLy(desired_pos_back_null:end) = [];
t(t_back_null:end) = [];

MAV1_pose_X(1:pos_front_null) = [];
MAV1_pose_Y(1:pos_front_null) = [];
MAV1_pose_Z(1:pos_front_null) = [];
MAV2_pose_X(1:pos_front_null) = [];
MAV2_pose_Y(1:pos_front_null) = [];
MAV2_pose_Z(1:pos_front_null) = [];
c2_X(1:pos_front_null) = [];
c2_Y(1:pos_front_null) = [];
c2_Z(1:pos_front_null) = [];
c2_desired_X(1:desired_pos_front_null) = [];
c2_desired_Y(1:desired_pos_front_null) = [];
%c2_desired_Z(1:desired_pos_front_null) = [];
v_hat(1:desired_pos_front_null) = [];
vd(1:desired_pos_front_null) = [];
eta1(1:desired_pos_front_null) = [];
omega(1:desired_pos_front_null) = [];
omega_d(1:desired_pos_front_null) = [];
eta2(1:desired_pos_front_null) = [];
FLx(1:desired_pos_front_null) = [];
FLy(1:desired_pos_front_null) = [];
t(1:t_front_null) = [];
t = t - t(1);

t_ = linspace(0, time_duration, length(MAV1_pose_X));
t_c2 = linspace(0, time_duration, length(c2_X));
t_c2_desired = linspace(0, time_duration, length(c2_desired_X));

%RMSE

d_size = size(t_c2,2);
X_sum = zeros(1,d_size);
Y_sum = zeros(1,d_size);

polyfit_X = polyfit(t_c2_desired.',c2_desired_X,8);
polyfit_Y = polyfit(t_c2_desired.',c2_desired_Y,8);
X_fit = polyval(polyfit_X,t_c2);
Y_fit = polyval(polyfit_Y,t_c2);

for i = 1:d_size
    X_sum(1,i) = X_fit(i) - c2_X(i);
    Y_sum(1,i) = Y_fit(i) - c2_Y(i);
end

X_RMSE = sqrt(sum(X_sum.^2)/d_size);
Y_RMSE = sqrt(sum(Y_sum.^2)/d_size);
RMSE = sqrt((sum(X_sum.^2) + sum(Y_sum.^2))/d_size);

% figure
% set(subplot(211), 'Position', [0.15, 0.69, 0.77, 0.23])
% plot(t_c2, X_fit)
% y_label = ylabel('X (m)', 'rotation', 90);
% set(y_label, 'Units', 'Normalized', 'Position', [-0.07, 0.43]);
% xlim([0 t_c2(end)])
% title("Position of C_{2}", 'Fontsize', 11);
% 
% set(subplot(212), 'Position', [0.15, 0.4, 0.77, 0.23])
% plot(t_c2, Y_fit)
% y_label = ylabel('Y (m)', 'rotation', 90);
% set(y_label, 'Units', 'Normalized', 'Position', [-0.07, 0.43]);
% xlim([0 t_c2(end)])

% figure
% set(subplot(311), 'Position', [0.15, 0.69, 0.77, 0.23])
% plot(t_, MAV1_pose_X)
% y_label = ylabel({'$X$ ($\mathrm{m}$)'}, 'Interpreter', 'latex', 'rotation', 0);
% set(y_label, 'Units', 'Normalized', 'Position', [-0.12, 0.43]);
% xlim([0 t_(end)])
% title("Position of MAV1", 'Fontsize', 11);
% 
% set(subplot(312), 'Position', [0.15, 0.4, 0.77, 0.23])
% plot(t_, MAV1_pose_Y)
% y_label = ylabel({'$Y$ ($\mathrm{m}$)'}, 'Interpreter', 'latex', 'rotation', 0);
% set(y_label, 'Units', 'Normalized', 'Position', [-0.12, 0.43]);
% xlim([0 t_(end)])
% 
% set(subplot(313), 'Position', [0.15, 0.1, 0.77, 0.23])
% plot(t_, MAV1_pose_Z)
% y_label = ylabel({'$Z$ ($\mathrm{m}$)'}, 'Interpreter', 'latex', 'rotation', 0);
% set(y_label, 'Units', 'Normalized', 'Position', [-0.12, 0.43]);
% xlim([0 t_(end)])
% xlabel('Time (sec)', 'Fontsize', 11)
% 
% figure
% set(subplot(311), 'Position', [0.15, 0.69, 0.77, 0.23])
% plot(t_, MAV2_pose_X)
% y_label = ylabel({'$X$ ($\mathrm{m}$)'}, 'Interpreter', 'latex', 'rotation', 0);
% set(y_label, 'Units', 'Normalized', 'Position', [-0.12, 0.43]);
% xlim([0 t_(end)])
% title("Position of MAV2", 'Fontsize', 11);
% 
% set(subplot(312), 'Position', [0.15, 0.4, 0.77, 0.23])
% plot(t_, MAV2_pose_Y)
% y_label = ylabel({'$Y$ ($\mathrm{m}$)'}, 'Interpreter', 'latex', 'rotation', 0);
% set(y_label, 'Units', 'Normalized', 'Position', [-0.12, 0.43]);
% xlim([0 t_(end)])
% 
% set(subplot(313), 'Position', [0.15, 0.1, 0.77, 0.23])
% plot(t_, MAV2_pose_Z)
% y_label = ylabel({'$Z$ ($\mathrm{m}$)'}, 'Interpreter', 'latex', 'rotation', 0);
% set(y_label, 'Units', 'Normalized', 'Position', [-0.12, 0.43]);
% xlim([0 t_(end)])
% xlabel('Time (sec)', 'Fontsize', 11)
% 
% plot c2
% figure
% set(subplot(311), 'Position', [0.15, 0.69, 0.77, 0.23])
% plot(t_c2, c2_X)
% y_label = ylabel({'$X$ ($\mathrm{m}$)'}, 'Interpreter', 'latex', 'rotation', 0);
% set(y_label, 'Units', 'Normalized', 'Position', [-0.12, 0.43]);
% xlim([0 t_c2(end)])
% title("Position of C_{2}", 'Fontsize', 11);
% 
% set(subplot(312), 'Position', [0.15, 0.4, 0.77, 0.23])
% plot(t_c2, c2_Y)
% y_label = ylabel({'$Y$ ($\mathrm{m}$)'}, 'Interpreter', 'latex', 'rotation', 0);
% set(y_label, 'Units', 'Normalized', 'Position', [-0.12, 0.43]);
% xlim([0 t_c2(end)])
% 
% set(subplot(313), 'Position', [0.15, 0.1, 0.77, 0.23])
% plot(t_c2, c2_Z)
% y_label = ylabel({'$Z$ ($\mathrm{m}$)'}, 'Interpreter', 'latex', 'rotation', 0);
% set(y_label, 'Units', 'Normalized', 'Position', [-0.12, 0.43]);
% xlim([0 t_c2(end)])
% xlabel('Time (sec)', 'Fontsize', 11)
% 
% plot desired c2
% figure
% set(subplot(311), 'Position', [0.15, 0.69, 0.77, 0.23])
% plot(t_c2_desired, c2_desired_X)
% y_label = ylabel({'$X$ ($\mathrm{m}$)'}, 'Interpreter', 'latex', 'rotation', 0);
% set(y_label, 'Units', 'Normalized', 'Position', [-0.07, 0.43]);
% xlim([0 t_c2_desired(end)])
% title("Position of C2", 'Fontsize', 11);
% 
% set(subplot(312), 'Position', [0.15, 0.4, 0.77, 0.23])
% plot(t_c2_desired, c2_desired_Y)
% y_label = ylabel({'$Y$ ($\mathrm{m}$)'}, 'Interpreter', 'latex', 'rotation', 0);
% set(y_label, 'Units', 'Normalized', 'Position', [-0.07, 0.43]);
% xlim([0 t_c2_desired(end)])
% 
% set(subplot(313), 'Position', [0.15, 0.1, 0.77, 0.23])
% plot(t_c2_desired, c2_desired_Z)
% y_label = ylabel({'$Z$ ($\mathrm{m}$)'}, 'Interpreter', 'latex', 'rotation', 0);
% set(y_label, 'Units', 'Normalized', 'Position', [-0.07, 0.43]);
% xlim([0 t_c2_desired(end)])
% xlabel('Time (sec)', 'Fontsize', 11)

% compare
% figure
% set(subplot(211), 'Position', [0.15, 0.69, 0.77, 0.23])
% plot(t_c2, c2_X)
% hold on
% plot(t_c2_desired, c2_desired_X)
% hold on
% plot(t_c2, X_sum)
% y_label = ylabel('X (m)', 'rotation', 90);
% set(y_label, 'Units', 'Normalized', 'Position', [-0.07, 0.43]);
% xlim([0 t_c2(end)])
% legend('$x_{c_2}$', '$x_r$', '$x_e$', 'Interpreter', 'latex')
% title("Trajectory S", 'Fontsize', 11);
% 
% set(subplot(212), 'Position', [0.15, 0.4, 0.77, 0.23])
% plot(t_c2, c2_Y)
% hold on
% plot(t_c2_desired, c2_desired_Y)
% hold on
% plot(t_c2, Y_sum)
% y_label = ylabel('Y (m)', 'rotation', 90);
% set(y_label, 'Units', 'Normalized', 'Position', [-0.07, 0.43]);
% legend('$y_{c_2}$', '$y_r$', '$y_e$', 'Interpreter', 'latex')
% xlim([0 t_c2(end)])
% 
% % set(subplot(313), 'Position', [0.15, 0.1, 0.77, 0.23])
% % plot(t_c2, c2_Z)
% % hold on
% % plot(t_c2_desired, c2_desired_Z)
% % y_label = ylabel({'$Z$ ($\mathrm{m}$)'}, 'Interpreter', 'latex', 'rotation', 0);
% % set(y_label, 'Units', 'Normalized', 'Position', [-0.12, 0.43]);
% % xlim([0 t_c2(end)])
% xlabel('Time (sec)', 'Fontsize', 11)

% 2D
figure
plot(c2_X, c2_Y, '->', 'Linewidth', 2.5, 'MarkerIndices',1:10:length(c2_Y))
hold on
plot(c2_desired_X, c2_desired_Y, '--o', 'Linewidth', 2.5, 'MarkerIndices',1:10:length(c2_Y))
grid on
scatter_x = [-1.2 -0.8 0.0 0.4];
scatter_y = [0.9 0.85 0.15 0.1];
s = scatter(scatter_x,scatter_y);
s.LineWidth = 0.6;
s.MarkerEdgeColor = 'k';
s.MarkerFaceColor = 'g';
% for k=1:numel(scatter_x)
%       text(scatter_x(k)-0.15,scatter_y(k)-0.05,['(' num2str(scatter_x(k)) ',' num2str(scatter_y(k)) ')'])
% end
text(scatter_x(1)-0.05,scatter_y(1)+0.06,['(' num2str(scatter_x(1)) ', ' num2str(scatter_y(1)) ')'])
text(scatter_x(2)-0.1,scatter_y(2)+0.06,['(' num2str(scatter_x(2)) ', ' num2str(scatter_y(2)) ')'])
text(scatter_x(3),scatter_y(3)+0.05,['(' num2str(scatter_x(3)) ', ' num2str(scatter_y(3)) ')'])
text(scatter_x(4)-0.1,scatter_y(4)+0.05,['(' num2str(scatter_x(4)) ', ' num2str(scatter_y(4)) ')'])
legend('Actual trajectory', 'Desired trajectory', 'Waypoints', 'Interpreter', 'latex')
title("Trajectory S", 'Fontsize', 11);
xlabel('X (m)', 'Fontsize', 11)
ylabel('Y (m)', 'Fontsize', 11)
% 
% % figure 12
% figure
% plot(t_c2_desired, v_hat, 'r')
% hold on
% plot(t_c2_desired, vd, 'b--')
% hold on
% plot(t_c2_desired, eta1, 'm-.')
% grid on
% % ylim([-1.5 1.5])
% xlim([0, t(end)])
% y_label = ylabel('Velocity (m/s)', 'rotation', 90);
% set(y_label, 'Units', 'Normalized', 'Position', [-0.06, 0.47]);
% xlabel('Time (sec)', 'Fontsize', 11)
% legend('$\hat{v}$', '$v_d$', '$\eta_1$', 'Interpreter', 'latex')
% title('figure 12', 'Fontsize', 11)

% % figure 13
% figure
% plot(t_c2_desired, omega, 'r')
% hold on
% plot(t_c2_desired, omega_d, 'b--')
% hold on
% plot(t_c2_desired, eta2, 'm-.')
% grid on
% % ylim([-1.5 1.5])
% xlim([0, t(end)])
% y_label = ylabel('Angular rate (rad/s)', 'rotation', 90);
% set(y_label, 'Units', 'Normalized', 'Position', [-0.07, 0.47]);
% xlabel('Time (sec)', 'Fontsize', 11)
% legend('$\omega$', '$\omega_d$', '$\eta_2$', 'Interpreter', 'latex')
% title('figure 13', 'Fontsize', 11)
% 
% % figure 14
% figure
% plot(t_c2_desired, FLx, 'b')
% grid on
% ylim([-3 3])
% xlim([0, t(end)])
% y_label = ylabel('Force (N)', 'rotation', 90);
% set(y_label, 'Units', 'Normalized', 'Position', [-0.07, 0.47]);
% xlabel('Time (sec)', 'Fontsize', 11)
% legend('$F_{L_x}$', 'Interpreter', 'latex')
% title('figure 14', 'Fontsize', 11)
% 
% % figure 15
% figure
% plot(t_c2_desired, FLy, 'b')
% grid on
% ylim([-10 15])
% xlim([0, t(end)])
% y_label = ylabel('Force (N)', 'rotation', 90, 'Fontsize', 11);
% set(y_label, 'Units', 'Normalized', 'Position', [-0.07, 0.47]);
% xlabel('Time (sec)', 'Fontsize', 11)
% legend('$F_{L_y}$', 'Interpreter', 'latex')
% title('figure 15', 'Fontsize', 11)
