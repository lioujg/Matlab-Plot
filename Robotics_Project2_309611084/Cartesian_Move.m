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


tacc = 0.2;
T = 0.5;
r = (T - tacc) / T;

[x, y, z, psi, theta, phi] = callback_D1(A,B);
Dr = callback_Dr(x, y, z, psi, theta, phi, r);
Ap = A * Dr;

% calculate deltaB and deltaC
[X_a, Y_a, Z_a, Psi_a, theta_A, Phi_a] = callback_D1(B, Ap);
[X_c, Y_c, Z_c, Psi_c, theta_C, Phi_c] = callback_D1(B, C);

delta_C = [X_c Y_c Z_c];

if abs(rad2deg(Psi_c-Psi_a)) > 90
    Psi_a = Psi_a + pi;
    if Psi_a > pi
        Psi_a = Psi_a - 2 * pi;
    end
    theta_A = -theta_A;
end



Z_Axis = [];
a = [];
p = [];
v = [];

for t = -0.5 : 0.002 : 0.5
    if t < -0.2
        h = (t + 0.5) / T;
        Dr = callback_Dr(x, y, z, psi, theta, phi, h);
        Dv = [x / T ; y / T ; z / T ; 0];
        
        v = [v, A(1:3,:)*Dv];
        a = [a, zeros(3,1)];
        p = [p, A(1:3,:)*Dr(:,4)];
        
        tmp = A * Dr * [0 0 0.1 1]';
        Z_Axis = [Z_Axis tmp(1:3,1)];
        
    elseif t > 0.2
        h = t / T;
        Dv = [delta_C(1) / T;delta_C(2) / T;delta_C(3) / T;0];
        Dr = callback_Dr(X_c, Y_c, Z_c, Psi_c, theta_C, Phi_c, h);        

        v = [v, B(1:3,:) * Dv];
        p = [p, B(1:3,:) * Dr(:,4)];
        a = [a, zeros(3,1)];
        
        Z_Axis = [Z_Axis tmp(1:3,1)];
        tmp = B * Dr * [0 0 0.1 1]';
    else
        h = (t + tacc) / (2 * tacc);
        Dp = [((X_c * tacc / T + X_a) * (2-h) * h^2 - 2*X_a) * h + X_a;
              ((Y_c * tacc / T + Y_a) * (2-h) * h^2 - 2*Y_a) * h + Y_a;
              ((Z_c * tacc / T + Z_a) * (2-h) * h^2 - 2*Z_a) * h + Z_a;
              (Psi_c-Psi_a) * h + Psi_a;
              ((theta_C * tacc / T + theta_A) * (2 - h) * h^2 - 2 * theta_A) * h + theta_A;
              ((Phi_c * tacc / T + Phi_a) * (2 - h) * h^2 - 2 * Phi_a) * h + Phi_a];
        Dv = [((X_c * tacc / T + X_a) * (1.5 - h) * 2 * h^2 - X_a) / tacc;
              ((Y_c * tacc / T + Y_a) * (1.5 - h) * 2 * h^2 - Y_a) / tacc;
              ((Z_c * tacc / T + Z_a) * (1.5 - h) * 2 * h^2 - Z_a) / tacc;0];
        Da = [(X_c * tacc / T + X_a) * (1 - h) * 3 * h / tacc^2;
              (Y_c * tacc / T + Y_a) * (1 - h) * 3 * h / tacc^2;
              (Z_c * tacc / T + Z_a) * (1 - h) * 3 * h / tacc^2;0];

        Dr = callback_Dr(Dp(1), Dp(2), Dp(3), Dp(4), Dp(5), Dp(6), 1);
          
        p = [p, B(1:3,:) * Dr(:,4)];
        v = [v, B(1:3,:) * Dv];
        a = [a, B(1:3,:) * Da];
        
        tmp = B * Dr * [0 0 0.1 1]';
        Z_Axis = [Z_Axis tmp(1:3,1)];
    end    
end



% show the position, velocity, acceleration of x,y,z
t = -0.5:0.002:0.5;
figure

subplot(331), plot(t,p(1,:))
title('Position of x')
subplot(332), plot(t,v(1,:))
title('Velocity of x')
subplot(333), plot(t,a(1,:))
title('Acceleration of x')

subplot(334), plot(t,p(2,:))
title('Position of y'), ylabel('Position(m)')
subplot(335), plot(t,v(2,:))
title('Velocity of y'), ylabel('Velocity(m/s)')
subplot(336), plot(t,a(2,:))
title('Acceleration of y'), ylabel('Acceleration(m/s^2)')

subplot(337), plot(t,p(3,:))
title('Position of z'), xlabel('Time(s)')
subplot(338), plot(t,v(3,:))
title('Velocity of z'), xlabel('Time(s)')
subplot(339), plot(t,a(3,:))
title('Acceleration of z'), xlabel('Time(s)')

figure
% show the moving path
plot3(p(1,:), p(2,:), p(3,:))
hold on

% show the A B C and their orientation
Ax = A * [0.1    0    0   1]';
Ay = A * [  0  0.1    0   1]';
Az = A * [  0    0  0.1   1]';

plot3([A(1,4) Ax(1)], [A(2,4) Ax(2)], [A(3,4) Ax(3)], 'r', 'LineWidth', 2)
plot3([A(1,4) Ay(1)], [A(2,4) Ay(2)], [A(3,4) Ay(3)], 'g', 'LineWidth', 2)
plot3([A(1,4) Az(1)], [A(2,4) Az(2)], [A(3,4) Az(3)], 'b', 'LineWidth', 2)

Bx = B * [0.1    0    0   1]';
By = B * [  0  0.1    0   1]';
Bz = B * [  0    0  0.1   1]';

plot3([B(1,4) Bx(1)], [B(2,4) Bx(2)], [B(3,4) Bx(3)], 'r', 'LineWidth', 2)
plot3([B(1,4) By(1)], [B(2,4) By(2)], [B(3,4) By(3)], 'g', 'LineWidth', 2)
plot3([B(1,4) Bz(1)], [B(2,4) Bz(2)], [B(3,4) Bz(3)], 'b', 'LineWidth', 2)

Cx = C * [0.1    0    0   1]';
Cy = C * [  0  0.1    0   1]';
Cz = C * [  0    0  0.1   1]';

plot3([C(1,4) Cx(1)], [C(2,4) Cx(2)], [C(3,4) Cx(3)], 'r', 'LineWidth', 2)
plot3([C(1,4) Cy(1)], [C(2,4) Cy(2)], [C(3,4) Cy(3)], 'g', 'LineWidth', 2)
plot3([C(1,4) Cz(1)], [C(2,4) Cz(2)], [C(3,4) Cz(3)], 'b', 'LineWidth', 2)

scatter3(A(1,4), A(2,4), A(3,4), [], 'k', 'filled')
text(A(1,4)-0.01, A(2,4)-0.01, A(3,4)-0.01, 'A(0.2,0.3,0.2)')
scatter3(B(1,4), B(2,4), B(3,4), [], 'k', 'filled')
text(B(1,4)-0.01, B(2,4)-0.01, B(3,4)-0.01, 'B(-0.1,0.15,0.3)')
scatter3(C(1,4), C(2,4), C(3,4), [], 'k', 'filled')
text(C(1,4)-0.01, C(2,4)-0.01, C(3,4)-0.01, 'C(-0.25,0.1,-0.2)')

xlabel('x(m)'), ylabel('y(m)'), zlabel('z(m)')
title('3D path of Cartesian Motion')
axis equal
set(gca,'XGrid','on'), set(gca,'YGrid','on'), set(gca,'ZGrid','on');
hold off

% show the moving path
figure
plot3(p(1,:), p(2,:), p(3,:), 'LineWidth', 2, 'color', [138 43 226]/255)
hold on

% show the A B C and their orientation
Ax = A*[0.1 0 0 1]';
Ay = A*[0 0.1 0 1]';
Az = A*[0 0 0.1 1]';

plot3([A(1,4) Ax(1)], [A(2,4) Ax(2)], [A(3,4) Ax(3)], 'r', 'LineWidth', 2)
plot3([A(1,4) Ay(1)], [A(2,4) Ay(2)], [A(3,4) Ay(3)], 'g', 'LineWidth', 2)
plot3([A(1,4) Az(1)], [A(2,4) Az(2)], [A(3,4) Az(3)], 'b', 'LineWidth', 2)

Bx = B*[0.1 0 0 1]';
By = B*[0 0.1 0 1]';
Bz = B*[0 0 0.1 1]';

plot3([B(1,4) Bx(1)], [B(2,4) Bx(2)], [B(3,4) Bx(3)], 'r', 'LineWidth', 2)
plot3([B(1,4) By(1)], [B(2,4) By(2)], [B(3,4) By(3)], 'g', 'LineWidth', 2)
plot3([B(1,4) Bz(1)], [B(2,4) Bz(2)], [B(3,4) Bz(3)], 'b', 'LineWidth', 2)

Cx = C*[0.1 0 0 1]';
Cy = C*[0 0.1 0 1]';
Cz = C*[0 0 0.1 1]';

plot3([C(1,4) Cx(1)], [C(2,4) Cx(2)], [C(3,4) Cx(3)], 'r', 'LineWidth', 2)
plot3([C(1,4) Cy(1)], [C(2,4) Cy(2)], [C(3,4) Cy(3)], 'g', 'LineWidth', 2)
plot3([C(1,4) Cz(1)], [C(2,4) Cz(2)], [C(3,4) Cz(3)], 'b', 'LineWidth', 2)

scatter3(A(1,4), A(2,4), A(3,4), [], 'k', 'filled')
text(A(1,4)-0.01, A(2,4)-0.01, A(3,4)-0.01, 'A(0.2,0.3,0.2)')
scatter3(B(1,4), B(2,4), B(3,4), [], 'k', 'filled')
text(B(1,4)-0.01, B(2,4)-0.01, B(3,4)-0.01, 'B(-0.1,0.15,0.3)')
scatter3(C(1,4), C(2,4), C(3,4), [], 'k', 'filled')
text(C(1,4)-0.01, C(2,4)-0.01, C(3,4)-0.01, 'C(-0.25,0.1,-0.2)')


% show the z-axis of end-effector
for i = 1:length(Z_Axis)
    plot3([p(1,i) Z_Axis(1,i)], [p(2,i) Z_Axis(2,i)], [p(3,i) Z_Axis(3,i)]);
end

xlabel('x(m)'), ylabel('y(m)'), zlabel('z(m)')
title('3D path of Cartesian Motion')
axis equal
set(gca,'XGrid','on'), set(gca,'YGrid','on'), set(gca,'ZGrid','on');
hold off

function [x, y, z, psi, theta, phi] = callback_D1(pos1, pos2)

x = pos1(:,1)' * (pos2(:,4) - pos1(:,4));
y = pos1(:,2)' * (pos2(:,4) - pos1(:,4));
z = pos1(:,3)' * (pos2(:,4) - pos1(:,4));
psi = atan2(pos1(:,2)'*pos2(:,3), pos1(:,1)'*pos2(:,3));
theta = atan2(sqrt((pos1(:,1)'*pos2(:,3))^2 + (pos1(:,2)'*pos2(:,3))^2), pos1(:,3)'*pos2(:,3));

phi_s = -sin(psi)*cos(psi)*(1-cos(theta))*pos1(:,1)'*pos2(:,1) ...
       +(cos(psi)^2*(1-cos(theta))+cos(theta))*pos1(:,2)'*pos2(:,1) ...
       - sin(psi)*sin(theta)*pos1(:,3)'*pos2(:,1);
phi_c = -sin(psi)*cos(psi)*(1-cos(theta))*pos1(:,1)'*pos2(:,2) ...
       +(cos(psi)^2*(1-cos(theta))+cos(theta))*pos1(:,2)'*pos2(:,2) ...
       - sin(psi)*sin(theta)*pos1(:,3)'*pos2(:,2);
phi = atan2(phi_s, phi_c);
end

function Dr = callback_Dr(x,y,z,psi,theta,phi,r)

Dr = zeros(4,4);
Dr(:,4) = [r*x;r*y;r*z;1];
Dr(:,3) = [cos(psi)*sin(r*theta);sin(psi)*sin(r*theta);cos(r*theta);0];
Dr(:,2) = [-sin(r*theta)*(sin(psi)^2*(1-cos(r*theta))+cos(r*theta)) + cos(r*phi)*(-sin(psi)*cos(psi)*(1-cos(r*theta)));
           -sin(r*theta)*(-sin(psi)*cos(psi)*(1-cos(r*theta))) + cos(r*phi)*(cos(psi)^2*(1-cos(r*theta))+cos(r*theta));
           -sin(r*theta)*(-cos(psi)*sin(r*theta)) + cos(r*theta)*(-sin(psi)*sin(r*theta));
           0];
Dr(1:3,1) = cross(Dr(1:3,2), Dr(1:3,3));
end