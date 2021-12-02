clear all; close all;clc

% A = [0  0  1 0.1;
%     -1  0  0 0.5;
%      0 -1  0 0.3;
%      0  0  0  1];
%  
% B = [0  0  1 0.3;
%      0  1  0 0.3;
%     -1  0  0 0.2;
%      0  0  0  1];
%  
% C = [0  1  0 -0.3;
%      0  0 -1 -0.25;
%     -1  0  0  0.25;
%      0  0  0   1];

A = [0  1  0   0.2;
    -1  0  0   0.3;
     0  0  1   0.2;
     0  0  0  0.01  ];
B = [ 0  0 -1   -0.1;
     -1  0  0   0.15;
      0  1  0    0.3;
      0  0  0   0.01  ];
C = [ 1  0  0  -0.25;
      0 -1  0    0.1;
      0  0 -1   -0.2;
      0  0  0   0.01  ];
tacc = 0.2;
T = 0.5;
r = (T-tacc)/T;
% find the point at which start to accelerate 
[x, y, z, psi, theta, phi] = calD1(A,B);
Dr = calDr(x, y, z, psi, theta, phi, r);
Ap = A*Dr;

% calculate deltaB and deltaC
[x_A, y_A, z_A, psi_A, theta_A, phi_A] = calD1(B, Ap);
[x_C, y_C, z_C, psi_C, theta_C, phi_C] = calD1(B, C);

delta_C = [x_C y_C z_C];

if abs(rad2deg(psi_C-psi_A)) > 90
    psi_A = psi_A + pi;
    if psi_A > pi
        psi_A = psi_A - 2*pi;
    end
    theta_A = -theta_A;
end


% start motion from A to C pass nearby B and find their z-axis
p = []; v = [];a = [];
zAxis = [];
for t = -0.5:0.002:0.5
    if t < -0.2
        h = (t+0.5)/T;
        Dr = calDr(x, y, z, psi, theta, phi, h);
        Dv = [x/T;y/T;z/T;0];

        p = [p, A(1:3,:)*Dr(:,4)];
        v = [v, A(1:3,:)*Dv];
        a = [a, zeros(3,1)];
        
        temp = A*Dr*[0 0 0.1 1]';
        zAxis = [zAxis temp(1:3,1)];
    elseif t > 0.2
        h = t/T;
        Dr = calDr(x_C, y_C, z_C, psi_C, theta_C, phi_C, h);
        Dv = [delta_C(1)/T;delta_C(2)/T;delta_C(3)/T;0];

        p = [p, B(1:3,:)*Dr(:,4)];
        v = [v, B(1:3,:)*Dv];
        a = [a, zeros(3,1)];
        
        temp = B*Dr*[0 0 0.1 1]';
        zAxis = [zAxis temp(1:3,1)];
    else
        h = (t+tacc)/(2*tacc);
        Dp = [((x_C*tacc/T+x_A)*(2-h)*h^2-2*x_A)*h + x_A;
              ((y_C*tacc/T+y_A)*(2-h)*h^2-2*y_A)*h + y_A;
              ((z_C*tacc/T+z_A)*(2-h)*h^2-2*z_A)*h + z_A;
              (psi_C-psi_A)*h+psi_A;
              ((theta_C*tacc/T+theta_A)*(2-h)*h^2-2*theta_A)*h + theta_A;
              ((phi_C*tacc/T+phi_A)*(2-h)*h^2-2*phi_A)*h + phi_A];
        Dv = [((x_C*tacc/T+x_A)*(1.5-h)*2*h^2-x_A)/tacc;
              ((y_C*tacc/T+y_A)*(1.5-h)*2*h^2-y_A)/tacc;
              ((z_C*tacc/T+z_A)*(1.5-h)*2*h^2-z_A)/tacc;0];
        Da = [(x_C*tacc/T+x_A)*(1-h)*3*h/tacc^2;
              (y_C*tacc/T+y_A)*(1-h)*3*h/tacc^2;
              (z_C*tacc/T+z_A)*(1-h)*3*h/tacc^2;0];

        Dr = calDr(Dp(1), Dp(2), Dp(3), Dp(4), Dp(5), Dp(6), 1);
          
        p = [p, B(1:3,:)*Dr(:,4)];
        v = [v, B(1:3,:)*Dv];
        a = [a, B(1:3,:)*Da];
        
        temp = B*Dr*[0 0 0.1 1]';
        zAxis = [zAxis temp(1:3,1)];
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
text(A(1,4)-0.01, A(2,4)-0.01, A(3,4)-0.01, 'A(0.1,0.5,0.3)')
scatter3(B(1,4), B(2,4), B(3,4), [], 'k', 'filled')
text(B(1,4)-0.01, B(2,4)-0.01, B(3,4)-0.01, 'B(0.3,0.3,0.2)')
scatter3(C(1,4), C(2,4), C(3,4), [], 'k', 'filled')
text(C(1,4)-0.01, C(2,4)-0.01, C(3,4)-0.01, 'C(-0.3,-0.25,0.25)')

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
text(A(1,4)-0.01, A(2,4)-0.01, A(3,4)-0.01, 'A(0.1,0.5,0.3)')
scatter3(B(1,4), B(2,4), B(3,4), [], 'k', 'filled')
text(B(1,4)-0.01, B(2,4)-0.01, B(3,4)-0.01, 'B(0.3,0.3,0.2)')
scatter3(C(1,4), C(2,4), C(3,4), [], 'k', 'filled')
text(C(1,4)-0.01, C(2,4)-0.01, C(3,4)-0.01, 'C(-0.3,-0.25,0.25)')


% show the z-axis of end-effector
for i = 1:length(zAxis)
    plot3([p(1,i) zAxis(1,i)], [p(2,i) zAxis(2,i)], [p(3,i) zAxis(3,i)]);
end

xlabel('x(m)'), ylabel('y(m)'), zlabel('z(m)')
title('3D path of Cartesian Motion')
axis equal
set(gca,'XGrid','on'), set(gca,'YGrid','on'), set(gca,'ZGrid','on');
hold off
