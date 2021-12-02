close all;
csv_select = "serial_log0521";
data = readmatrix(csv_select);
dt = 1/20;

if csv_select == "serial_log0521"
    data_start_null = 1100;
    data_end_null = 1800;
    pos_Jxx = 0.036;
    pos_Jyy = 0.032;
    pos_Jzz = 0.028;
    fig_title = "$Estimation$ $of$ $Force$";
end
data(1:data_start_null, :) = [];
data(data_end_null:end, :) = [];

subplot(2,1,1)

t = data(:, 4) - data(1, 4);
Fx = data(:, 1);
Fy = data(:, 2);
Fz = data(:, 3);
trigger_x = data(:, 5);
trigger_y = data(:, 6);

figure(1)
plot(t, Fx)
hold on
plot(t, Fy)
hold on
plot(t, Fz)
hold on
%plot(t, trigger_x)
%hold on
yline(1.0,'--.r','F_{upper}','LabelVerticalAlignment','middle');
yline(0.6,'--.b','F_{lower}','LabelVerticalAlignment','middle');
yline(-0.6,'--.b','-F_{lower}','LabelVerticalAlignment','middle');
yline(-1.0,'--.r','-F_{upper}','LabelVerticalAlignment','middle');


grid on

legend('$\hat{F}_{x}$', '$\hat{F}_{y}$', '$\hat{F}_{z}$', '$Upper$', '$Lower$', 'Interpreter', 'latex')
xlim([0, t(end)])
ylim([-1.5, 1.5])
xlabel('$Time(sec)$', 'Interpreter', 'latex');
y_label = ylabel('$F$', 'Interpreter', 'latex', 'rotation', 0);
set(y_label, 'Units', 'Normalized', 'Position', [-0.04, 0.47]);
title(fig_title, 'Interpreter', 'latex');

subplot(2,1,2)
plot(t, trigger_x)
xlim([0, t(end)])
ylim([-0.5, 1.5])
hold on
plot(t, trigger_y)
xlim([0, t(end)])
ylim([-0.5, 1.5])

