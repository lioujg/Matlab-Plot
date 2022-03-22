% Plot
% read experiment data from bag file and plot the trajectory
close all

% parameters for plotting
bag_select = "UCT_traj.bag";


% read data from bag file
bag = rosbag(bag_select);

% obtain trajectory
trajectory_bag = select(bag, 'topic', '/drone1/desired_position');
trajectory_msgStructs = readMessages(trajectory_bag, 'DataFormat', 'struct');
traj_pose_X = cellfun(@(m) double(m.X), trajectory_msgStructs);
traj_pose_Y = cellfun(@(m) double(m.Y), trajectory_msgStructs);
traj_pose_Z = cellfun(@(m) double(m.Z), trajectory_msgStructs);

% delete useless points
% traj_pose_X(pos_back_null:end) = [];
% traj_pose_Y(pos_back_null:end) = [];
% traj_pose_Z(pos_back_null:end) = [];
% 
% 
% traj_pose_X(1:pos_front_null) = [];
% traj_pose_Y(1:pos_front_null) = [];
% traj_pose_Z(1:pos_front_null) = [];


figure
plot(traj_pose_X, traj_pose_Y, '--', 'Linewidth', 2.5)
axis equal
xlim([-7 7])
title("Trajectory", 'Fontsize', 11);
% legend('Actual trajectory', 'Desired trajectory', 'Waypoints', 'Interpreter', 'latex')
xlabel('X (m)', 'Fontsize', 11)
ylabel('Y (m)', 'Fontsize', 11)

