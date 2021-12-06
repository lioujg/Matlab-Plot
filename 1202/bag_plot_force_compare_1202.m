% Plot
% read experiment data from bag file and plot the trajectory
close all
clear all

% parameters for plotting
bag_select = "2021-12-02-13-38-16.bag";

if bag_select == "2021-12-02-13-38-16.bag"
%         pos_front_null = 1;
%         pos_back_null = 1425;
%         t_front_null = pos_front_null*2;
%         t_back_null = pos_back_null*2;
elseif  bag_select == "2021-12-02-11-56-25.bag"
%         pos_front_null = 1;
%         pos_back_null = 1124;
%         t_front_null = pos_front_null*2;
%         t_back_null = pos_back_null*2;
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

% obtain payload force sensor measurement by Robot 1
payload_bag = select(bag, 'topic', 'payload_joint_1_ft_sensor');
Robot_1_msgStructs = readMessages(payload_bag, 'DataFormat', 'struct');
Robot_1_fm_x = cellfun(@(m) double(m.Wrench.Force.X), Robot_1_msgStructs);
Robot_1_fm_y = cellfun(@(m) double(m.Wrench.Force.Y), Robot_1_msgStructs);
Robot_1_fm_z = cellfun(@(m) double(m.Wrench.Force.Z), Robot_1_msgStructs);
Robot_1_tm_x = cellfun(@(m) double(m.Wrench.Torque.X), Robot_1_msgStructs);
Robot_1_tm_y = cellfun(@(m) double(m.Wrench.Torque.Y), Robot_1_msgStructs);
Robot_1_tm_z = cellfun(@(m) double(m.Wrench.Torque.Z), Robot_1_msgStructs);

% obtain payload force sensor measurement by Robot 2
payload_bag = select(bag, 'topic', 'payload_joint_2_ft_sensor');
Robot_2_msgStructs = readMessages(payload_bag, 'DataFormat', 'struct');
Robot_2_fm_x = cellfun(@(m) double(m.Wrench.Force.X), Robot_2_msgStructs);
Robot_2_fm_y = cellfun(@(m) double(m.Wrench.Force.Y), Robot_2_msgStructs);
Robot_2_fm_z = cellfun(@(m) double(m.Wrench.Force.Z), Robot_2_msgStructs);
Robot_2_tm_x = cellfun(@(m) double(m.Wrench.Torque.X), Robot_2_msgStructs);
Robot_2_tm_y = cellfun(@(m) double(m.Wrench.Torque.Y), Robot_2_msgStructs);
Robot_2_tm_z = cellfun(@(m) double(m.Wrench.Torque.Z), Robot_2_msgStructs);

% obtain payload force computed by Robot 1
payload_bag = select(bag, 'topic', '/Robot1/robot_1_wrench');
Robot_1_msgStructs = readMessages(payload_bag, 'DataFormat', 'struct');
Robot_1_fc_x = cellfun(@(m) double(m.Force.X), Robot_1_msgStructs);
Robot_1_fc_y = cellfun(@(m) double(m.Force.Y), Robot_1_msgStructs);
Robot_1_fc_z = cellfun(@(m) double(m.Force.Z), Robot_1_msgStructs);
Robot_1_tc_x = cellfun(@(m) double(m.Torque.X), Robot_1_msgStructs);
Robot_1_tc_y = cellfun(@(m) double(m.Torque.Y), Robot_1_msgStructs);
Robot_1_tc_z = cellfun(@(m) double(m.Torque.Z), Robot_1_msgStructs);

% obtain payload force computed by Robot 2
payload_bag = select(bag, 'topic', '/Robot2/robot_2_wrench');
Robot_2_msgStructs = readMessages(payload_bag, 'DataFormat', 'struct');
Robot_2_fc_x = cellfun(@(m) double(m.Force.X), Robot_2_msgStructs);
Robot_2_fc_y = cellfun(@(m) double(m.Force.Y), Robot_2_msgStructs);
Robot_2_fc_z = cellfun(@(m) double(m.Force.Z), Robot_2_msgStructs);
Robot_2_tc_x = cellfun(@(m) double(m.Torque.X), Robot_2_msgStructs);
Robot_2_tc_y = cellfun(@(m) double(m.Torque.Y), Robot_2_msgStructs);
Robot_2_tc_z = cellfun(@(m) double(m.Torque.Z), Robot_2_msgStructs);

pos_front_null = 1;
t_front_null = 2;
pos_back_null = size(Robot_1_fm_x);
t_back_null = pos_back_null*2;

% delete useless points
Robot_1_fm_x(pos_back_null:end) = [];
Robot_1_fm_y(pos_back_null:end) = [];
Robot_1_fm_z(pos_back_null:end) = [];
Robot_1_tm_x(pos_back_null:end) = [];
Robot_1_tm_y(pos_back_null:end) = [];
Robot_1_tm_z(pos_back_null:end) = [];
Robot_2_fm_x(pos_back_null:end) = [];
Robot_2_fm_y(pos_back_null:end) = [];
Robot_2_fm_z(pos_back_null:end) = [];
Robot_2_tm_x(pos_back_null:end) = [];
Robot_2_tm_y(pos_back_null:end) = [];
Robot_2_tm_z(pos_back_null:end) = [];
Robot_1_fc_x(pos_back_null:end) = [];
Robot_1_fc_y(pos_back_null:end) = [];
Robot_1_fc_z(pos_back_null:end) = [];
Robot_1_tc_x(pos_back_null:end) = [];
Robot_1_tc_y(pos_back_null:end) = [];
Robot_1_tc_z(pos_back_null:end) = [];
Robot_2_fc_x(pos_back_null:end) = [];
Robot_2_fc_y(pos_back_null:end) = [];
Robot_2_fc_z(pos_back_null:end) = [];
Robot_2_tc_x(pos_back_null:end) = [];
Robot_2_tc_y(pos_back_null:end) = [];
Robot_2_tc_z(pos_back_null:end) = [];
t(t_back_null:end) = [];


Robot_1_fm_x(1:pos_front_null) = [];
Robot_1_fm_y(1:pos_front_null) = [];
Robot_1_fm_z(1:pos_front_null) = [];
Robot_1_tm_x(1:pos_front_null) = [];
Robot_1_tm_y(1:pos_front_null) = [];
Robot_1_tm_z(1:pos_front_null) = [];
Robot_2_fm_x(1:pos_front_null) = [];
Robot_2_fm_y(1:pos_front_null) = [];
Robot_2_fm_z(1:pos_front_null) = [];
Robot_2_tm_x(1:pos_front_null) = [];
Robot_2_tm_y(1:pos_front_null) = [];
Robot_2_tm_z(1:pos_front_null) = [];
Robot_1_fc_x(1:pos_front_null) = [];
Robot_1_fc_y(1:pos_front_null) = [];
Robot_1_fc_z(1:pos_front_null) = [];
Robot_1_tc_x(1:pos_front_null) = [];
Robot_1_tc_y(1:pos_front_null) = [];
Robot_1_tc_z(1:pos_front_null) = [];
Robot_2_fc_x(1:pos_front_null) = [];
Robot_2_fc_y(1:pos_front_null) = [];
Robot_2_fc_z(1:pos_front_null) = [];
Robot_2_tc_x(1:pos_front_null) = [];
Robot_2_tc_y(1:pos_front_null) = [];
Robot_2_tc_z(1:pos_front_null) = [];
t(1:t_front_null) = [];
t = t - t(1);

t_ = linspace(0, time_duration, length(Robot_1_fc_x));
m_t = linspace(0, time_duration, length(Robot_1_fm_x));

figure(1)
set(subplot(311), 'Position', [0.17, 0.7, 0.76, 0.2])
plot(m_t, Robot_1_fm_x, 'b', 'Linewidth', 2)
hold on
plot(t_, Robot_1_fc_x, 'r', 'Linewidth', 2)
grid on
% ylim([0.0 0.1])
xlim([0, t_(end)])
y_label = ylabel('$N$', 'Interpreter', 'latex', 'rotation', 0);
set(y_label, 'Units', 'Normalized', 'Position', [-0.13, 0.41]);
legend('$f_{xm}$', '$f_{xc}$', 'Interpreter', 'latex')
title('Force', 'Fontsize', 11)

set(subplot(312), 'Position', [0.17, 0.4, 0.76, 0.2])
plot(m_t, Robot_1_fm_y, 'b', 'Linewidth', 2)
hold on
plot(t_, Robot_1_fc_y, 'r', 'Linewidth', 2)
grid on
% ylim([0.1 0.9])
xlim([0, t_(end)])
y_label = ylabel('$N$', 'Interpreter', 'latex', 'rotation', 0);
set(y_label, 'Units', 'Normalized', 'Position', [-0.13, 0.41]);
legend('$f_{ym}$', '$f_{yc}$', 'Interpreter', 'latex')

set(subplot(313), 'Position', [0.17, 0.1, 0.76, 0.2])
plot(m_t, Robot_1_fm_z, 'b', 'Linewidth', 2)
hold on
plot(t_, Robot_1_fc_z, 'r', 'Linewidth', 2)
grid on
% ylim([-0.3 0.9])
xlim([0, t_(end)])
y_label = ylabel('$N$', 'Interpreter', 'latex', 'rotation', 0);
set(y_label, 'Units', 'Normalized', 'Position', [-0.13, 0.41]);
xlabel('Time (sec)', 'Fontsize', 11)
legend('$f_{zm}$', '$f_{zc}$', 'Interpreter', 'latex')

figure(2)
set(subplot(311), 'Position', [0.17, 0.7, 0.76, 0.2])
plot(m_t, Robot_1_tm_x, 'b', 'Linewidth', 2)
hold on
plot(t_, Robot_1_tc_x, 'r', 'Linewidth', 2)
grid on
% ylim([0.0 0.1])
xlim([0, t_(end)])
y_label = ylabel('$N$', 'Interpreter', 'latex', 'rotation', 0);
set(y_label, 'Units', 'Normalized', 'Position', [-0.13, 0.41]);
legend('$t_{xm}$', '$t_{xc}$', 'Interpreter', 'latex')
title('Torque', 'Fontsize', 11)

set(subplot(312), 'Position', [0.17, 0.4, 0.76, 0.2])
plot(m_t, Robot_1_tm_y, 'b', 'Linewidth', 2)
hold on
plot(t_, Robot_1_tc_y, 'r', 'Linewidth', 2)
grid on
% ylim([0.1 0.9])
xlim([0, t_(end)])
y_label = ylabel('$N$', 'Interpreter', 'latex', 'rotation', 0);
set(y_label, 'Units', 'Normalized', 'Position', [-0.13, 0.41]);
legend('$t_{ym}$', '$t_{yc}$', 'Interpreter', 'latex')

set(subplot(313), 'Position', [0.17, 0.1, 0.76, 0.2])
plot(m_t, Robot_1_tm_z, 'b', 'Linewidth', 2)
hold on
plot(t_, Robot_1_tc_z, 'r', 'Linewidth', 2)
grid on
% ylim([-0.3 0.9])
xlim([0, t_(end)])
y_label = ylabel('$N$', 'Interpreter', 'latex', 'rotation', 0);
set(y_label, 'Units', 'Normalized', 'Position', [-0.13, 0.41]);
xlabel('Time (sec)', 'Fontsize', 11)
legend('$t_{zm}$', '$t_{zc}$', 'Interpreter', 'latex')




figure(3)
set(subplot(311), 'Position', [0.17, 0.7, 0.76, 0.2])
plot(m_t, Robot_2_fm_x, 'b', 'Linewidth', 2)
hold on
plot(t_, Robot_2_fc_x, 'r', 'Linewidth', 2)
grid on
% ylim([0.0 0.1])
xlim([0, t_(end)])
y_label = ylabel('$N$', 'Interpreter', 'latex', 'rotation', 0);
set(y_label, 'Units', 'Normalized', 'Position', [-0.13, 0.41]);
legend('$f_{xm}$', '$f_{xc}$', 'Interpreter', 'latex')
title('Force', 'Fontsize', 11)

set(subplot(312), 'Position', [0.17, 0.4, 0.76, 0.2])
plot(m_t, Robot_2_fm_y, 'b', 'Linewidth', 2)
hold on
plot(t_, Robot_2_fc_y, 'r', 'Linewidth', 2)
grid on
% ylim([0.1 0.9])
xlim([0, t_(end)])
y_label = ylabel('$N$', 'Interpreter', 'latex', 'rotation', 0);
set(y_label, 'Units', 'Normalized', 'Position', [-0.13, 0.41]);
legend('$f_{ym}$', '$f_{yc}$', 'Interpreter', 'latex')

set(subplot(313), 'Position', [0.17, 0.1, 0.76, 0.2])
plot(m_t, Robot_2_fm_z, 'b', 'Linewidth', 2)
hold on
plot(t_, Robot_2_fc_z, 'r', 'Linewidth', 2)
grid on
% ylim([-0.3 0.9])
xlim([0, t_(end)])
y_label = ylabel('$N$', 'Interpreter', 'latex', 'rotation', 0);
set(y_label, 'Units', 'Normalized', 'Position', [-0.13, 0.41]);
xlabel('Time (sec)', 'Fontsize', 11)
legend('$f_{zm}$', '$f_{zc}$', 'Interpreter', 'latex')

figure(4)
set(subplot(311), 'Position', [0.17, 0.7, 0.76, 0.2])
plot(m_t, Robot_2_tm_x, 'b', 'Linewidth', 2)
hold on
plot(t_, Robot_2_tc_x, 'r', 'Linewidth', 2)
grid on
% ylim([0.0 0.1])
xlim([0, t_(end)])
y_label = ylabel('$N$', 'Interpreter', 'latex', 'rotation', 0);
set(y_label, 'Units', 'Normalized', 'Position', [-0.13, 0.41]);
legend('$t_{xm}$', '$t_{xc}$', 'Interpreter', 'latex')
title('Torque', 'Fontsize', 11)

set(subplot(312), 'Position', [0.17, 0.4, 0.76, 0.2])
plot(m_t, Robot_2_tm_y, 'b', 'Linewidth', 2)
hold on
plot(t_, Robot_2_tc_y, 'r', 'Linewidth', 2)
grid on
% ylim([0.1 0.9])
xlim([0, t_(end)])
y_label = ylabel('$N$', 'Interpreter', 'latex', 'rotation', 0);
set(y_label, 'Units', 'Normalized', 'Position', [-0.13, 0.41]);
legend('$t_{ym}$', '$t_{yc}$', 'Interpreter', 'latex')

set(subplot(313), 'Position', [0.17, 0.1, 0.76, 0.2])
plot(m_t, Robot_2_tm_z, 'b', 'Linewidth', 2)
hold on
plot(t_, Robot_2_tc_z, 'r', 'Linewidth', 2)
grid on
% ylim([-0.3 0.9])
xlim([0, t_(end)])
y_label = ylabel('$N$', 'Interpreter', 'latex', 'rotation', 0);
set(y_label, 'Units', 'Normalized', 'Position', [-0.13, 0.41]);
xlabel('Time (sec)', 'Fontsize', 11)
legend('$t_{zm}$', '$t_{zc}$', 'Interpreter', 'latex')
