% Plot
% read experiment data from bag file and plot the trajectory
close all
clear all

% parameters for plotting
bag_select = "without_ICL.bag";

if bag_select == "2021-12-21-17-32-56.bag"
    pos_back_null = 690;
elseif bag_select == "without_ICL.bag"
    pos_back_null = 800;
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

% obtain payload reference trajectory
payload_bag = select(bag, 'topic', '/payload/desired_trajectory');
Payload_msgStructs = readMessages(payload_bag, 'DataFormat', 'struct');
Payload_x_r = cellfun(@(m) double(m.Transforms.Translation.X), Payload_msgStructs);
Payload_y_r = cellfun(@(m) double(m.Transforms.Translation.Y), Payload_msgStructs);

% obtain payload trajectory
payload_bag = select(bag, 'topic', '/payload/position');
Payload_msgStructs = readMessages(payload_bag, 'DataFormat', 'struct');
Payload_x = cellfun(@(m) double(m.Pose.Pose.Position.X), Payload_msgStructs);
Payload_y = cellfun(@(m) double(m.Pose.Pose.Position.Y), Payload_msgStructs);

% obtain payload trajectory waypoints
waypoint_x = [0.5 3.5 13.5 3.5 -2.5 -12.5 -2.5 0.5];
waypoint_y = [0 5 0 -5 5 0 -5 0];

% obtain velocity error s calculated by Robot 1
Robot_1_bag = select(bag, 'topic', '/robot_1/s_norm');
Robot_1_msgStructs = readMessages(Robot_1_bag, 'DataFormat', 'struct');
Robot_1_s = cellfun(@(m) double(m.X), Robot_1_msgStructs);

% obtain velocity error s calculated by Robot 2(same as Robot 1)
Robot_2_bag = select(bag, 'topic', '/robot_2/s_norm');
Robot_2_msgStructs = readMessages(Robot_2_bag, 'DataFormat', 'struct');
Robot_2_s = cellfun(@(m) double(m.X), Robot_2_msgStructs);

% obtain payload position error and yaw angle error by Robot 1
Robot_1_bag = select(bag, 'topic', '/robot_1/position_error');
Robot_1_msgStructs = readMessages(Robot_1_bag, 'DataFormat', 'struct');
Robot_1_pos_err_x = cellfun(@(m) double(m.X), Robot_1_msgStructs);
Robot_1_pos_err_y = cellfun(@(m) double(m.Y), Robot_1_msgStructs);
Robot_1_yaw_err_z = cellfun(@(m) double(m.Theta), Robot_1_msgStructs);

% % obtain payload inertia estimated by Robot 2
% payload_bag = select(bag, 'topic', '/robot_2/estimated/inertia');
% Robot_2_msgStructs = readMessages(payload_bag, 'DataFormat', 'struct');
% Robot_2_inertia_Ixx = cellfun(@(m) double(m.X), Robot_2_msgStructs);
% Robot_2_inertia_Iyy = cellfun(@(m) double(m.Y), Robot_2_msgStructs);
% Robot_2_inertia_Izz = cellfun(@(m) double(m.Z), Robot_2_msgStructs);

% pos_front_null = 1;
% t_front_null = 2;
% pos_back_null = size(Robot_1_mass);
t_back_null = pos_back_null*2;

% delete useless points
Payload_x_r(pos_back_null:end) = [];
Payload_y_r(pos_back_null:end) = [];
Robot_1_s(pos_back_null:end) = [];
Robot_2_s(pos_back_null:end) = [];
Robot_1_pos_err_x(pos_back_null:end) = [];
Robot_1_pos_err_y(pos_back_null:end) = [];
Robot_1_yaw_err_z(pos_back_null:end) = [];
% Robot_1_inertia_Ixx(pos_back_null:end) = [];
% Robot_1_inertia_Iyy(pos_back_null:end) = [];
% Robot_1_inertia_Izz(pos_back_null:end) = [];
% Robot_2_inertia_Ixx(pos_back_null:end) = [];
% Robot_2_inertia_Iyy(pos_back_null:end) = [];
% Robot_2_inertia_Izz(pos_back_null:end) = [];
t(t_back_null:end) = [];
% 
% Payload_x_r(1:pos_front_null) = [];
% Payload_y_r(1:pos_front_null) = [];
% Robot_1_inertia_Ixx(1:pos_front_null) = [];
% Robot_1_inertia_Iyy(1:pos_front_null) = [];
% Robot_1_inertia_Izz(1:pos_front_null) = [];
% Robot_2_inertia_Ixx(1:pos_front_null) = [];
% Robot_2_inertia_Iyy(1:pos_front_null) = [];
% Robot_2_inertia_Izz(1:pos_front_null) = [];
% t(1:t_front_null) = [];
t = t - t(1);

t_ = linspace(0, time_duration, length(Robot_1_s));

figure(1)
subplot('Position', [0.17, 0.1, 0.76, 0.8]);
plot(Payload_x_r, Payload_y_r, 'r--', 'Linewidth', 2)
hold on
scatter(waypoint_x,waypoint_y,'filled','g', 'MarkerEdgeColor', 'k');
hold on
for k=1:numel(waypoint_x)
      text(waypoint_x(k)+0.3,waypoint_y(k),['(' num2str(waypoint_x(k)) ',' num2str(waypoint_y(k)) ')'])
end
hold on
plot(Payload_x, Payload_y, 'b');
grid on
y_label = ylabel('$Y$ (m)', 'Interpreter', 'latex', 'rotation', 0);
set(y_label, 'Units', 'Normalized', 'Position', [-0.1, 0.47]);
x_label = xlabel('$X$ (m)', 'Interpreter', 'latex', 'rotation', 0);
set(x_label, 'Units', 'Normalized');
legend('Desired trajectory', 'Waypoints', 'Actual trajectory', 'Location','north');
title('Trajectory', 'Fontsize', 11)

figure(2)
subplot('Position', [0.17, 0.1, 0.76, 0.8]);
plot(t_, Robot_1_s, 'b', 'Linewidth', 2)
hold on
% plot(t_, Robot_2_s, 'r--', 'Linewidth', 2)
grid on
% ylim([-0.5 4])
xlim([0, t_(end)])
y_label = ylabel('||s||', 'rotation', 0);
set(y_label, 'Units', 'Normalized', 'Position', [-0.08, 0.47]);
xlabel('Time (sec)', 'Fontsize', 11)
legend('$||s||$', 'Interpreter', 'latex')
title('||s||', 'Fontsize', 11)

figure(3)
set(subplot(311), 'Position', [0.17, 0.7, 0.76, 0.2])
plot(t_, Robot_1_pos_err_x, 'b', 'Linewidth', 2)
grid on
ylim([-2 2])
xlim([0, t_(end)])
y_label = ylabel('$x$ (m)', 'Interpreter', 'latex', 'rotation', 0);
set(y_label, 'Units', 'Normalized', 'Position', [-0.1, 0.41]);
% legend('$distance$', 'Interpreter', 'latex')
title('Position errors', 'Fontsize', 11)

set(subplot(312), 'Position', [0.17, 0.4, 0.76, 0.2])
plot(t_, Robot_1_pos_err_y, 'b', 'Linewidth', 2)
grid on
ylim([-2 2])
xlim([0, t_(end)])
y_label = ylabel('$y$ (m)', 'Interpreter', 'latex', 'rotation', 0);
set(y_label, 'Units', 'Normalized', 'Position', [-0.1, 0.41]);
% legend('$distance$', 'Interpreter', 'latex')

set(subplot(313), 'Position', [0.17, 0.1, 0.76, 0.2])
plot(t_, Robot_1_yaw_err_z, 'b', 'Linewidth', 2)
grid on
ylim([-pi/2 pi/2])
xlim([0, t_(end)])
y_label = ylabel('$yaw$', 'Interpreter', 'latex', 'rotation', 0);
set(y_label, 'Units', 'Normalized', 'Position', [-0.1, 0.41]);
xlabel('Time (sec)', 'Fontsize', 11)
% legend('$angle$', 'Interpreter', 'latex')