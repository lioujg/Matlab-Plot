% Plot
% read experiment data from bag file and plot the trajectory
close all

% parameters for plotting
bag_select = "2021-08-12-17-58-59.bag";

if bag_select == "2021-08-12-17-58-59.bag"
        length_of_t = 16184;
        pos_front_null = 1;
        pos_back_null = 970;
        t_front_null = pos_front_null*2;
        t_back_null = pos_back_null*2;
        desired_pos_front_null = 179;
        desired_pos_back_null = 500;
        c2_desired_Z = ones(desired_pos_back_null-desired_pos_front_null-1 ,1) * 0.36;
elseif  bag_select == "2021-08-12-18-07-20.bag"
        length_of_t = 16184;
        pos_front_null = 1;
        pos_back_null = 970;
        t_front_null = pos_front_null*2;
        t_back_null = pos_back_null*2;
        desired_pos_front_null = 196;
        desired_pos_back_null = 517;
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
t(1:t_front_null) = [];
t = t - t(1);

if  bag_select == "2021-08-12-18-07-20.bag"
    c2_X(478:480) = [0.65, 0.68, 0.71];
    c2_Y(478:480) = [-0.06, -0.05, -0.04];
    
elseif bag_select == "2021-08-12-17-58-59.bag"
    c2_X(494:510) = [0];
    c2_Y(494:510) = [0.05];
    c2_X(576:600) = [0.55];
    c2_Y(576:600) = [0.03];
end

t_ = linspace(0, time_duration, length(MAV1_pose_X));
t_c2 = linspace(0, time_duration, length(c2_X));
t_c2_desired = linspace(0, time_duration, length(c2_desired_X));

figure
set(subplot(311), 'Position', [0.15, 0.69, 0.77, 0.23])
plot(t_, MAV1_pose_X)
y_label = ylabel({'$X$ ($\mathrm{m}$)'}, 'Interpreter', 'latex', 'rotation', 0);
set(y_label, 'Units', 'Normalized', 'Position', [-0.12, 0.43]);
xlim([0 t_(end)])
title("Position of MAV1", 'Fontsize', 11);

set(subplot(312), 'Position', [0.15, 0.4, 0.77, 0.23])
plot(t_, MAV1_pose_Y)
y_label = ylabel({'$Y$ ($\mathrm{m}$)'}, 'Interpreter', 'latex', 'rotation', 0);
set(y_label, 'Units', 'Normalized', 'Position', [-0.12, 0.43]);
xlim([0 t_(end)])

set(subplot(313), 'Position', [0.15, 0.1, 0.77, 0.23])
plot(t_, MAV1_pose_Z)
y_label = ylabel({'$Z$ ($\mathrm{m}$)'}, 'Interpreter', 'latex', 'rotation', 0);
set(y_label, 'Units', 'Normalized', 'Position', [-0.12, 0.43]);
xlim([0 t_(end)])
xlabel('Time (sec)', 'Fontsize', 11)

figure
set(subplot(311), 'Position', [0.15, 0.69, 0.77, 0.23])
plot(t_, MAV2_pose_X)
y_label = ylabel({'$X$ ($\mathrm{m}$)'}, 'Interpreter', 'latex', 'rotation', 0);
set(y_label, 'Units', 'Normalized', 'Position', [-0.12, 0.43]);
xlim([0 t_(end)])
title("Position of MAV2", 'Fontsize', 11);

set(subplot(312), 'Position', [0.15, 0.4, 0.77, 0.23])
plot(t_, MAV2_pose_Y)
y_label = ylabel({'$Y$ ($\mathrm{m}$)'}, 'Interpreter', 'latex', 'rotation', 0);
set(y_label, 'Units', 'Normalized', 'Position', [-0.12, 0.43]);
xlim([0 t_(end)])

set(subplot(313), 'Position', [0.15, 0.1, 0.77, 0.23])
plot(t_, MAV2_pose_Z)
y_label = ylabel({'$Z$ ($\mathrm{m}$)'}, 'Interpreter', 'latex', 'rotation', 0);
set(y_label, 'Units', 'Normalized', 'Position', [-0.12, 0.43]);
xlim([0 t_(end)])
xlabel('Time (sec)', 'Fontsize', 11)

% plot c2
figure
set(subplot(311), 'Position', [0.15, 0.69, 0.77, 0.23])
plot(t_c2, c2_X)
y_label = ylabel({'$X$ ($\mathrm{m}$)'}, 'Interpreter', 'latex', 'rotation', 0);
set(y_label, 'Units', 'Normalized', 'Position', [-0.12, 0.43]);
xlim([0 t_c2(end)])
title("Position of C_{2}", 'Fontsize', 11);

set(subplot(312), 'Position', [0.15, 0.4, 0.77, 0.23])
plot(t_c2, c2_Y)
y_label = ylabel({'$Y$ ($\mathrm{m}$)'}, 'Interpreter', 'latex', 'rotation', 0);
set(y_label, 'Units', 'Normalized', 'Position', [-0.12, 0.43]);
xlim([0 t_c2(end)])

set(subplot(313), 'Position', [0.15, 0.1, 0.77, 0.23])
plot(t_c2, c2_Z)
y_label = ylabel({'$Z$ ($\mathrm{m}$)'}, 'Interpreter', 'latex', 'rotation', 0);
set(y_label, 'Units', 'Normalized', 'Position', [-0.12, 0.43]);
xlim([0 t_c2(end)])
xlabel('Time (sec)', 'Fontsize', 11)

% plot desired c2
figure
set(subplot(311), 'Position', [0.15, 0.69, 0.77, 0.23])
plot(t_c2_desired, c2_desired_X)
y_label = ylabel({'$X$ ($\mathrm{m}$)'}, 'Interpreter', 'latex', 'rotation', 0);
set(y_label, 'Units', 'Normalized', 'Position', [-0.12, 0.43]);
xlim([0 t_c2_desired(end)])
title("Position of C2", 'Fontsize', 11);

set(subplot(312), 'Position', [0.15, 0.4, 0.77, 0.23])
plot(t_c2_desired, c2_desired_Y)
y_label = ylabel({'$Y$ ($\mathrm{m}$)'}, 'Interpreter', 'latex', 'rotation', 0);
set(y_label, 'Units', 'Normalized', 'Position', [-0.12, 0.43]);
xlim([0 t_c2_desired(end)])

set(subplot(313), 'Position', [0.15, 0.1, 0.77, 0.23])
plot(t_c2_desired, c2_desired_Z)
y_label = ylabel({'$Z$ ($\mathrm{m}$)'}, 'Interpreter', 'latex', 'rotation', 0);
set(y_label, 'Units', 'Normalized', 'Position', [-0.12, 0.43]);
xlim([0 t_c2_desired(end)])
xlabel('Time (sec)', 'Fontsize', 11)

% compare
figure
set(subplot(311), 'Position', [0.15, 0.69, 0.77, 0.23])
plot(t_c2, c2_X)
hold on
plot(t_c2_desired, c2_desired_X)
y_label = ylabel({'$X$ ($\mathrm{m}$)'}, 'Interpreter', 'latex', 'rotation', 0);
set(y_label, 'Units', 'Normalized', 'Position', [-0.12, 0.43]);
xlim([0 t_c2(end)])
title("Position of C2", 'Fontsize', 11);

set(subplot(312), 'Position', [0.15, 0.4, 0.77, 0.23])
plot(t_c2, c2_Y)
hold on
plot(t_c2_desired, c2_desired_Y)
y_label = ylabel({'$Y$ ($\mathrm{m}$)'}, 'Interpreter', 'latex', 'rotation', 0);
set(y_label, 'Units', 'Normalized', 'Position', [-0.12, 0.43]);
xlim([0 t_c2(end)])

set(subplot(313), 'Position', [0.15, 0.1, 0.77, 0.23])
plot(t_c2, c2_Z)
hold on
plot(t_c2_desired, c2_desired_Z)
y_label = ylabel({'$Z$ ($\mathrm{m}$)'}, 'Interpreter', 'latex', 'rotation', 0);
set(y_label, 'Units', 'Normalized', 'Position', [-0.12, 0.43]);
xlim([0 t_c2(end)])
xlabel('Time (sec)', 'Fontsize', 11)

figure
plot(c2_X, c2_Y)
hold on
plot(c2_desired_X, c2_desired_Y)
