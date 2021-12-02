clear all
close all
clc

A = [0  1  0   20;
    -1  0  0   30;
     0  0  1   20;
     0  0  0    1  ];
B = [ 0  0 -1   -10;
     -1  0  0    15;
      0  1  0    30;
      0  0  0     1  ];
C = [ 1  0  0    -25;
      0 -1  0     10;
      0  0 -1    -20;
      0  0  0      1  ];

unit_matrix = [1 0 0    0;
               0 1 0    0;
               0 0 1    0;
               0 0 0 0.01  ];
           
A = A * unit_matrix
B = B * unit_matrix
C = C * unit_matrix
  
  
Theta_A = [31.900670   32.474962   -34.610195   0.000000   2.135233   -121.900670]';
Theta_B = [67.948868   -2.529005   -44.425928   105.455397   74.075319   -45.219766]';
Theta_C = [124.599928   -28.219289   -127.988642   -0.000000   -23.792068   124.599928]';


tacc = 0.2;
T = 0.5;
r = (T - tacc) / T;

Theta_AP = r * (Theta_B - Theta_A) + Theta_A;


Delta_C = Theta_C - Theta_B;
Delta_B = Theta_AP - Theta_B;


Theta_a = [];
Theta_p = [];
Theta_v = [];

for t = -0.5:0.002:0.5
    if t < -0.2
        Theta_p = [Theta_p (Theta_B - Theta_A) * (t + 0.5) / T + Theta_A];
        Theta_v = [Theta_v (Theta_B - Theta_A) / T];
        Theta_a = [Theta_a zeros(6,1)];
    elseif t > 0.2
        Theta_p = [Theta_p Delta_C * t / T + Theta_B];
        Theta_v = [Theta_v Delta_C / T];
        Theta_a = [Theta_a zeros(6,1)];
    else
        h = (t + tacc) / (2 * tacc);
        
        Theta_p = [Theta_p ((Delta_C * tacc / T + Delta_B) * (2 - h) * h^2 - 2 * Delta_B) * h + Delta_B + Theta_B];
        Theta_v = [Theta_v ((Delta_C * tacc / T + Delta_B) * (1.5 - h) * 2 * h^2 - Delta_B) / tacc];
        Theta_a = [Theta_a (Delta_C * tacc / T + Delta_B) * (1 - h) * 3 * h / tacc^2];        
    end
end





t = -0.5:0.002:0.5;
t1 = -0.5:0.002:-0.2-0.002;
t2 = -0.2:0.002:0.2;
t3 = 0.2+0.002:0.002:0.5;

figure
for i = 1:6
    subplot(7,3,1 + 3 * (i - 1)), plot(t,Theta_p(i,:))
    ylabel(['\Theta', num2str(i)])
    subplot(7,3,2 + 3 * (i - 1)), plot(t,Theta_v(i,:))
    subplot(7,3,3 + 3 * (i - 1)), plot(t,Theta_a(i,:))
end

subplot(731), title('Joint Value')
subplot(732), title('Velocity')
subplot(733), title('Acceleration')

Z_Axis = [];
p = [];

for i = Theta_p
    T = forward(i);    
    temp = T * [0 0 0.1 1]';
    Z_Axis = [Z_Axis temp(1:3,1)];
    p = [p T(1:3,4)];

end

figure
plot3(p(1,1:length(t1)), p(2,1:length(t1)), p(3,1:length(t1)), 'color', [138 43 226]/255)
hold on
plot3(p(1,length(t1)+1:length(t1)+length(t2)+1), p(2,length(t1)+1:length(t1)+length(t2)+1), p(3,length(t1)+1:length(t1)+length(t2)+1), 'm')
plot3(p(1,end:-1:end-length(t3)+1), p(2,end:-1:end-length(t3)+1), p(3,end:-1:end-length(t3)+1), 'c')

Ax = A * [0.1 0 0 1]';
Ay = A * [0 0.1 0 1]';
Az = A * [0 0 0.1 1]';

plot3([A(1,4) Ax(1)], [A(2,4) Ax(2)], [A(3,4) Ax(3)], 'r', 'LineWidth', 2)
plot3([A(1,4) Ay(1)], [A(2,4) Ay(2)], [A(3,4) Ay(3)], 'g', 'LineWidth', 2)
plot3([A(1,4) Az(1)], [A(2,4) Az(2)], [A(3,4) Az(3)], 'b', 'LineWidth', 2)

Bx = B * [0.1 0 0 1]';
By = B * [0 0.1 0 1]';
Bz = B * [0 0 0.1 1]';

plot3([B(1,4) Bx(1)], [B(2,4) Bx(2)], [B(3,4) Bx(3)], 'r', 'LineWidth', 2)
plot3([B(1,4) By(1)], [B(2,4) By(2)], [B(3,4) By(3)], 'g', 'LineWidth', 2)
plot3([B(1,4) Bz(1)], [B(2,4) Bz(2)], [B(3,4) Bz(3)], 'b', 'LineWidth', 2)

Cx = C * [0.1 0 0 1]';
Cy = C * [0 0.1 0 1]';
Cz = C * [0 0 0.1 1]';

plot3([C(1,4) Cx(1)], [C(2,4) Cx(2)], [C(3,4) Cx(3)], 'r', 'LineWidth', 2)
plot3([C(1,4) Cy(1)], [C(2,4) Cy(2)], [C(3,4) Cy(3)], 'g', 'LineWidth', 2)
plot3([C(1,4) Cz(1)], [C(2,4) Cz(2)], [C(3,4) Cz(3)], 'b', 'LineWidth', 2)

scatter3(A(1,4), A(2,4), A(3,4), [], 'k', 'filled')
text(A(1,4)+0.05, A(2,4)-0.05, A(3,4)-0.01, 'A(0.2,0.3,0.2)')
scatter3(B(1,4), B(2,4), B(3,4), [], 'k', 'filled')
text(B(1,4)-0.05, B(2,4)-0.1, B(3,4)-0.05, 'B(-0.1,0.15,0.3)')
scatter3(C(1,4), C(2,4), C(3,4), [], 'k', 'filled')
text(C(1,4), C(2,4)-0.05, C(3,4)+0.1, 'C(-0.25,0.1,-0.2)')

hold off, axis equal
xlabel('x(m)'), ylabel('y(m)'), zlabel('z(m)')
title('3D path of Joint Motion')
set(gca,'XGrid','on'), set(gca,'YGrid','on'), set(gca,'ZGrid','on');

figure
plot3(p(1,:),p(2,:),p(3,:), 'LineWidth', 2, 'color', [138 43 226]/255), hold on

Ax = A * [0.1 0 0 1]';
Ay = A * [0 0.1 0 1]';
Az = A * [0 0 0.1 1]';

plot3([A(1,4) Ax(1)], [A(2,4) Ax(2)], [A(3,4) Ax(3)], 'r', 'LineWidth', 2)
plot3([A(1,4) Ay(1)], [A(2,4) Ay(2)], [A(3,4) Ay(3)], 'g', 'LineWidth', 2)
plot3([A(1,4) Az(1)], [A(2,4) Az(2)], [A(3,4) Az(3)], 'b', 'LineWidth', 2)

Bx = B * [0.1 0 0 1]';
By = B * [0 0.1 0 1]';
Bz = B * [0 0 0.1 1]';

plot3([B(1,4) Bx(1)], [B(2,4) Bx(2)], [B(3,4) Bx(3)], 'r', 'LineWidth', 2)
plot3([B(1,4) By(1)], [B(2,4) By(2)], [B(3,4) By(3)], 'g', 'LineWidth', 2)
plot3([B(1,4) Bz(1)], [B(2,4) Bz(2)], [B(3,4) Bz(3)], 'b', 'LineWidth', 2)

Cx = C * [0.1 0 0 1]';
Cy = C * [0 0.1 0 1]';
Cz = C * [0 0 0.1 1]';

plot3([C(1,4) Cx(1)], [C(2,4) Cx(2)], [C(3,4) Cx(3)], 'r', 'LineWidth', 2)
plot3([C(1,4) Cy(1)], [C(2,4) Cy(2)], [C(3,4) Cy(3)], 'g', 'LineWidth', 2)
plot3([C(1,4) Cz(1)], [C(2,4) Cz(2)], [C(3,4) Cz(3)], 'b', 'LineWidth', 2)

scatter3(A(1,4), A(2,4), A(3,4), [], 'k', 'filled')
text(A(1,4)+0.05, A(2,4)-0.05, A(3,4)-0.01, 'A(0.2,0.3,0.2)')
scatter3(B(1,4), B(2,4), B(3,4), [], 'k', 'filled')
text(B(1,4)-0.05, B(2,4)-0.1, B(3,4)-0.05, 'B(-0.1,0.15,0.3)')
scatter3(C(1,4), C(2,4), C(3,4), [], 'k', 'filled')
text(C(1,4), C(2,4)-0.05, C(3,4)+0.1, 'C(-0.25,0.1,-0.2)')

for i = 1:length(Z_Axis)
    plot3([p(1,i) Z_Axis(1,i)], [p(2,i) Z_Axis(2,i)], [p(3,i) Z_Axis(3,i)]);
end

xlabel('x(m)'), ylabel('y(m)'), zlabel('z(m)')
title('3D path of Joint Motion')
axis equal
set(gca,'XGrid','on'), set(gca,'YGrid','on'), set(gca,'ZGrid','on');



function T6 = forward(Theta)

Theta1 = Theta(1) / 180 * pi;
Theta2 = Theta(2) / 180 * pi;
Theta3 = Theta(3) / 180 * pi;
Theta4 = Theta(4) / 180 * pi;
Theta5 = Theta(5) / 180 * pi;
Theta6 = Theta(6) / 180 * pi;

a2 = 0.432;
a3 = -0.02;

%%transformatiom matrix A1~A6

A1 = [  cos(Theta1)              0    -sin(Theta1)                  0;
        sin(Theta1)              0     cos(Theta1)                  0;
                  0             -1               0                  0;
                  0              0               0                  1  ];
             
A2 = [  cos(Theta2)    -sin(Theta2)              0     a2*cos(Theta2);
        sin(Theta2)     cos(Theta2)              0     a2*sin(Theta2);
                  0               0              1                  0;
                  0               0              0                  1  ];

A3 = [  cos(Theta3)              0     sin(Theta3)     a3*cos(Theta3);
        sin(Theta3)              0    -cos(Theta3)     a3*sin(Theta3);
                  0              1               0              0.149;
                  0              0               0                  1  ];
             
A4 = [  cos(Theta4)              0     -sin(Theta4)                0;
        sin(Theta4)              0      cos(Theta4)                0;
                  0             -1                0            0.433;
                  0              0                0                1  ];
             
A5 = [  cos(Theta5)              0      sin(Theta5)                0;
        sin(Theta5)              0     -cos(Theta5)                0;
                  0              1                0                0;
                  0              0                0                1  ];
             
A6 = [  cos(Theta6)    -sin(Theta6)              0                 0;
        sin(Theta6)     cos(Theta6)              0                 0;
                  0               0              1                 0;
                  0               0              0                 1  ];

T6 = A1 * A2 * A3 * A4 * A5 * A6;
end
