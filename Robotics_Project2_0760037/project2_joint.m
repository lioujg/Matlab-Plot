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

thetaA = [31.900670   32.474962   -34.610195   0.000000   2.135233   -121.900670]';
thetaB = [67.948868   -2.529005   -44.425928   105.455397   74.075319   -45.219766]';
thetaC = [124.599928   -28.219289   -127.988642   -0.000000   -23.792068   124.599928]';



tacc = 0.2;
T = 0.5;
r = (T-tacc)/T;
% find the point at which start to accelerate 
thetaAp = thetaA + (thetaB-thetaA)*r;

% calculate deltaB and deltaC
deltaC = thetaC-thetaB;
deltaB = thetaAp-thetaB;

% start motion from A to C pass nearby B and find their z-axis
theta_p = [];theta_v = [];theta_a = [];
for t = -0.5:0.002:0.5
    if t < -0.2
        theta_p = [theta_p (thetaB-thetaA)*(t+0.5)/T + thetaA];
        theta_v = [theta_v (thetaB-thetaA)/T];
        theta_a = [theta_a zeros(6,1)];
    elseif t > 0.2
        theta_p = [theta_p deltaC*t/T+thetaB];
        theta_v = [theta_v deltaC/T];
        theta_a = [theta_a zeros(6,1)];
    else
        h = (t+tacc)/(2*tacc);
        
        theta_p = [theta_p ((deltaC*tacc/T+deltaB)*(2-h)*h^2-2*deltaB)*h + deltaB + thetaB];
        theta_v = [theta_v ((deltaC*tacc/T+deltaB)*(1.5-h)*2*h^2-deltaB)/tacc];
        theta_a = [theta_a (deltaC*tacc/T+deltaB)*(1-h)*3*h/tacc^2];        
    end
end




% plot the result
t = -0.5:0.002:0.5;
t1 = -0.5:0.002:-0.2-0.002;
t2 = -0.2:0.002:0.2;
t3 = 0.2+0.002:0.002:0.5;

figure
for i = 1:6
    subplot(7,3,1+3*(i-1)), plot(t,theta_p(i,:))
    ylabel(['\theta', num2str(i)])
    subplot(7,3,2+3*(i-1)), plot(t,theta_v(i,:))
    subplot(7,3,3+3*(i-1)), plot(t,theta_a(i,:))
end
subplot(731), title('Joint Value')
subplot(732), title('Velocity')
subplot(733), title('Acceleration')

p = [];zAxis = [];
for i = theta_p
    T = forward_kinematic(i);
    p = [p T(1:3,4)];
    temp = T*[0 0 0.1 1]';
    zAxis = [zAxis temp(1:3,1)];
end

figure
plot3(p(1,1:length(t1)), p(2,1:length(t1)), p(3,1:length(t1)), 'color', [138 43 226]/255)
hold on
plot3(p(1,length(t1)+1:length(t1)+length(t2)+1), p(2,length(t1)+1:length(t1)+length(t2)+1), p(3,length(t1)+1:length(t1)+length(t2)+1), 'm')
plot3(p(1,end:-1:end-length(t3)+1), p(2,end:-1:end-length(t3)+1), p(3,end:-1:end-length(t3)+1), 'c')

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
text(A(1,4)+0.05, A(2,4)-0.05, A(3,4)-0.01, 'A(0.1,0.5,0.3)')
scatter3(B(1,4), B(2,4), B(3,4), [], 'k', 'filled')
text(B(1,4)-0.05, B(2,4)-0.1, B(3,4)-0.05, 'B(0.3,0.3,0.2)')
scatter3(C(1,4), C(2,4), C(3,4), [], 'k', 'filled')
text(C(1,4), C(2,4)-0.05, C(3,4)+0.1, 'C(-0.3,-0.25,0.25)')

hold off, axis equal
xlabel('x(m)'), ylabel('y(m)'), zlabel('z(m)')
title('3D path of Joint Motion')
set(gca,'XGrid','on'), set(gca,'YGrid','on'), set(gca,'ZGrid','on');

figure
plot3(p(1,:),p(2,:),p(3,:), 'LineWidth', 2, 'color', [138 43 226]/255), hold on

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
text(A(1,4)+0.05, A(2,4)-0.05, A(3,4)-0.01, 'A(0.1,0.5,0.3)')
scatter3(B(1,4), B(2,4), B(3,4), [], 'k', 'filled')
text(B(1,4)-0.05, B(2,4)-0.1, B(3,4)-0.05, 'B(0.3,0.3,0.2)')
scatter3(C(1,4), C(2,4), C(3,4), [], 'k', 'filled')
text(C(1,4), C(2,4)-0.05, C(3,4)+0.1, 'C(-0.3,-0.25,0.25)')

for i = 1:length(zAxis)
    plot3([p(1,i) zAxis(1,i)], [p(2,i) zAxis(2,i)], [p(3,i) zAxis(3,i)]);
end

xlabel('x(m)'), ylabel('y(m)'), zlabel('z(m)')
title('3D path of Joint Motion')
axis equal
set(gca,'XGrid','on'), set(gca,'YGrid','on'), set(gca,'ZGrid','on');







