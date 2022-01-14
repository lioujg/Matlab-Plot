% Plot
% read experiment data from bag file and plot the trajectory
close all
% clear all

% parameters for plotting
bag_select = "2022-01-14-16-24-22.bag";

if bag_select == "without_ICL.bag"
%         pos_front_null = 1;
%         pos_back_null = 1425;
%         t_front_null = pos_front_null*2;
%         t_back_null = pos_back_null*2;
elseif  bag_select == "2021-12-21-17-32-56.bag"
%         pos_front_null = 1;
%         pos_back_null = 1124;
%         t_front_null = pos_front_null*2;
%         t_back_null = pos_back_null*2;
elseif  bag_select == "2022-01-14-15-29-02.bag"
elseif  bag_select == "2022-01-14-15-50-09.bag"
elseif  bag_select == "2022-01-14-16-04-14.bag"
elseif  bag_select == "2022-01-14-16-24-22.bag"
end

% set ground truth
ground_truth_m = 5/2;
ground_truth_Ixx = 0.052083333/2;
ground_truth_Iyy = 1.692708333/2;
ground_truth_Izz = 1.692708333/2;

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

% obtain payload mass estimated by Robot 1
payload_bag = select(bag, 'topic', '/robot_1/estimated');
Robot_1_msgStructs = readMessages(payload_bag, 'DataFormat', 'struct');
Robot_1_mass = cellfun(@(m) double(m.M), Robot_1_msgStructs);
Robot_1_inertia_Ixx = cellfun(@(m) double(m.Ixx), Robot_1_msgStructs);
Robot_1_inertia_Iyy = cellfun(@(m) double(m.Iyy), Robot_1_msgStructs);
Robot_1_inertia_Izz = cellfun(@(m) double(m.Izz), Robot_1_msgStructs);

% obtain payload mass estimated by Robot 2
payload_bag = select(bag, 'topic', '/robot_2/estimated');
Robot_2_msgStructs = readMessages(payload_bag, 'DataFormat', 'struct');
Robot_2_mass = cellfun(@(m) double(m.M), Robot_2_msgStructs);
Robot_2_inertia_Ixx = cellfun(@(m) double(m.Ixx), Robot_1_msgStructs);
Robot_2_inertia_Iyy = cellfun(@(m) double(m.Iyy), Robot_1_msgStructs);
Robot_2_inertia_Izz = cellfun(@(m) double(m.Izz), Robot_1_msgStructs);

pos_front_null = 1;
t_front_null = 2;
pos_back_null = size(Robot_1_mass);
t_back_null = pos_back_null*2;

% delete useless points
Robot_1_mass(pos_back_null:end) = [];
Robot_2_mass(pos_back_null:end) = [];
Robot_1_inertia_Ixx(pos_back_null:end) = [];
Robot_1_inertia_Iyy(pos_back_null:end) = [];
Robot_1_inertia_Izz(pos_back_null:end) = [];
Robot_2_inertia_Ixx(pos_back_null:end) = [];
Robot_2_inertia_Iyy(pos_back_null:end) = [];
Robot_2_inertia_Izz(pos_back_null:end) = [];
t(t_back_null:end) = [];

Robot_1_mass(1:pos_front_null) = [];
Robot_2_mass(1:pos_front_null) = [];
Robot_1_inertia_Ixx(1:pos_front_null) = [];
Robot_1_inertia_Iyy(1:pos_front_null) = [];
Robot_1_inertia_Izz(1:pos_front_null) = [];
Robot_2_inertia_Ixx(1:pos_front_null) = [];
Robot_2_inertia_Iyy(1:pos_front_null) = [];
Robot_2_inertia_Izz(1:pos_front_null) = [];
t(1:t_front_null) = [];
t = t - t(1);

t_ = linspace(0, time_duration, length(Robot_1_mass));

figure(1)
subplot('Position', [0.17, 0.1, 0.76, 0.8]);
plot(t_, Robot_1_mass, 'b-*', 'Linewidth', 1.5, 'MarkerIndices',1:30:length(t_))
hold on
plot(t_, Robot_2_mass, 'r-.', 'Linewidth', 2)
hold on
yline(ground_truth_m,'--','ground truth');
grid on
ylim([-0.5 4])
xlim([0, t_(end)])
y_label = ylabel('$m$ (kg)', 'Interpreter', 'latex', 'rotation', 0);
set(y_label, 'Units', 'Normalized', 'Position', [-0.13, 0.47]);
xlabel('Time (sec)', 'Fontsize', 11)
legend('$m_1$', '$m_2$', 'Interpreter', 'latex')
title('Mass', 'Fontsize', 11)

figure(2)
set(subplot(311), 'Position', [0.17, 0.7, 0.76, 0.2])
plot(t_, Robot_1_inertia_Ixx, 'b-*', 'Linewidth', 1.5, 'MarkerIndices',1:30:length(t_))
hold on
plot(t_, Robot_2_inertia_Ixx, 'r-.', 'Linewidth', 2)
hold on
yline(ground_truth_Ixx,'--','ground truth');
grid on
ylim([0.0 0.1])
xlim([0, t_(end)])
y_label = ylabel('$kg\cdot m^2$', 'Interpreter', 'latex', 'rotation', 0);
%y_label = ylabel('$inertia$', 'Interpreter', 'latex', 'rotation', 0);
set(y_label, 'Units', 'Normalized', 'Position', [-0.13, 0.41]);
legend('$I_{xx} 1$', '$I_{xx} 2$', 'Interpreter', 'latex')
title('Inertia', 'Fontsize', 11)

set(subplot(312), 'Position', [0.17, 0.4, 0.76, 0.2])
plot(t_, Robot_1_inertia_Iyy, 'b-*', 'Linewidth', 1.5, 'MarkerIndices',1:30:length(t_))
hold on
plot(t_, Robot_2_inertia_Iyy, 'r-.', 'Linewidth', 2)
hold on
yline(ground_truth_Iyy,'--','ground truth');
grid on
ylim([0.1 0.9])
xlim([0, t_(end)])
y_label = ylabel('$kg\cdot m^2$', 'Interpreter', 'latex', 'rotation', 0);
set(y_label, 'Units', 'Normalized', 'Position', [-0.13, 0.41]);
legend('$I_{yy} 1$', '$I_{yy} 2$', 'Interpreter', 'latex')

set(subplot(313), 'Position', [0.17, 0.1, 0.76, 0.2])
plot(t_, Robot_1_inertia_Izz, 'b-*', 'Linewidth', 1.5, 'MarkerIndices',1:30:length(t_))
hold on
plot(t_, Robot_2_inertia_Izz, 'r-.', 'Linewidth', 2)
hold on
yline(ground_truth_Izz,'--','ground truth');
grid on
ylim([-0.3 0.9])
xlim([0, t_(end)])
y_label = ylabel('$kg\cdot m^2$', 'Interpreter', 'latex', 'rotation', 0);
set(y_label, 'Units', 'Normalized', 'Position', [-0.13, 0.41]);
xlabel('Time (sec)', 'Fontsize', 11)
legend('$I_{zz} 1$', '$I_{zz} 2$', 'Interpreter', 'latex')