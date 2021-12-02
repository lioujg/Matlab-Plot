close all;
csv_select = "serial_log_fin";
data = readmatrix(csv_select);
dt = 1/20;

if csv_select == "serial_log_success"
    data_start_null = 380;
    data_end_null = 2544;
elseif csv_select == "serial_log_fin"
    data_start_null = 344;
    data_end_null = 1350;
end

data(data_end_null:end, :) = [];
data(1:data_start_null, :) = [];

t = data(:, 1) - data(1, 1);
UKF_norm = data(:, 2);
UKF_Fx = data(:, 3);
e = data(:, 4);
e_dot = data(:, 5);
Fx = data(:, 6);
Fy = data(:, 7);
trigger = data(:, 8);
trigger = (trigger-0.5)*100;
yaw = data(:, 9);

%figure(1)



subplot(4,1,1);
plot(t, UKF_norm)
hold on
plot(t, UKF_Fx)

legend('${F}_{norm}$', '$\hat{F}_{x}$', 'Interpreter', 'latex')
xlim([0, t(end)])
xlabel('$Time(sec)$', 'Interpreter', 'latex');
y_label = ylabel('$N$ , $N$', 'Interpreter', 'latex', 'rotation', 0);
set(y_label, 'Units', 'Normalized', 'Position', [-0.08, 0.47]);
title('$Body$ $x$ $direction$', 'Interpreter', 'latex','Color','b');


subplot(4,1,2);
plot(t, e)
hold on
plot(t, e_dot)

legend('$e$', '$\dot{e}$', 'Interpreter', 'latex')
xlim([0, t(end)])
xlabel('$Time(sec)$', 'Interpreter', 'latex');
y_label = ylabel('$N$ , $m$', 'Interpreter', 'latex', 'rotation', 0);
set(y_label, 'Units', 'Normalized', 'Position', [-0.08, 0.47]);
title('$Body$ $y$ $direction$', 'Interpreter', 'latex','Color','b');

subplot(4,1,3);
plot(t, trigger, 'LineWidth',1)
hold on
plot(t, yaw)

legend('$trigger$', '$yaw$', 'Interpreter', 'latex')
xlim([0, t(end)])
ylim([-180, 180])
xlabel('$Time(sec)$', 'Interpreter', 'latex');
y_label = ylabel('$trigger$ , $degree$', 'Interpreter', 'latex', 'rotation', 90);
set(y_label, 'Units', 'Normalized', 'Position', [-0.05, 0.47]);
title('$situation$', 'Interpreter', 'latex','Color','b');

subplot(4,1,4);
plot(t, Fx)
hold on
plot(t, Fy)


legend('${F}_{x}$', '${F}_{y}$', 'Interpreter', 'latex')
xlim([0, t(end)])
xlabel('$Time(sec)$', 'Interpreter', 'latex');
y_label = ylabel('$F$', 'Interpreter', 'latex', 'rotation', 0);
set(y_label, 'Units', 'Normalized', 'Position', [-0.05, 0.47]);
title('$Controller$ $force$', 'Interpreter', 'latex','Color','b');



