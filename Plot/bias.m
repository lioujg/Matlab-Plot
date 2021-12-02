close all;
csv_select = "serial_log_bias";
data = readmatrix(csv_select);
dt = 1/20;

if csv_select == "serial_log_bias"
    data_start_null = 1130;
    data_end_null = 1580;
end

data(data_end_null:end, :) = [];
data(1:data_start_null, :) = [];

bias_x = data(:, 1);
bias_y = data(:, 2);
bias_x_mean = mean(bias_x);
bias_y_mean = mean(bias_y);
% e = data(:, 4);
% e_dot = data(:, 5);
% Fx = data(:, 6);
% Fy = data(:, 7);
% trigger = data(:, 8);
% trigger = (trigger-0.5)*100;
% yaw = data(:, 9);

figure(1)

% subplot(4,1,1);
plot(bias_x)
hold on
plot(bias_y)
hold on
yline(bias_x_mean,'--.b','Mean_{x}','LabelVerticalAlignment','middle');
yline(bias_y_mean,'--.r','Mean_{y}','LabelVerticalAlignment','middle');

legend('${bias}_{x}$', '${bias}_{y}$', 'Interpreter', 'latex')
xlim([0, t(end)])
xlabel('$Time(sec)$', 'Interpreter', 'latex');
y_label = ylabel('$N$ , $N$', 'Interpreter', 'latex', 'rotation', 0);
set(y_label, 'Units', 'Normalized', 'Position', [-0.08, 0.47]);
title('$Body$ $x$ $direction$', 'Interpreter', 'latex','Color','b');



