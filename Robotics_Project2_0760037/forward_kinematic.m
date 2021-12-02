function T6 = forward_kinematic(theta)
% clear all; close all;clc

%%  parameters
%theta1 = 30.0003/180*pi;
%theta2 = 29.4022/180*pi;
%theta3 = 29.9860/180*pi;
%theta4 = 70.6104/180*pi;
%theta5 = 49.9976/180*pi;
%theta6 = 19.9979/180*pi;

%theta1 = 30.0003/180*pi;
%theta2 = 59.9900/180*pi;
%theta3 = -29.9860/180*pi;
%theta4 = 99.9947/180*pi;
%theta5 = 49.9976/180*pi;
%theta6 = 19.9979/180*pi;

% theta1 = 14.9985/180*pi;
% theta2 = 90.0043/180*pi;
% theta3 = -50.0115/180*pi;
% theta4 = 20.0084/180*pi;
% theta5 = 49.9993/180*pi;
% theta6 = -59.9980/180*pi;

theta1 = theta(1)/180*pi;
theta2 = theta(2)/180*pi;
theta3 = theta(3)/180*pi;
theta4 = theta(4)/180*pi;
theta5 = theta(5)/180*pi;
theta6 = theta(6)/180*pi;

%(78.6901  -21.9802  -30.5766   52.5567  168.6901         0)
%45.0000   12.2384  -88.9084   76.6700  135.0000   90.0000
%-140.1944    2.1035  -87.5413   85.4378   39.8056   90.0000
a1 = 0.12;
a2 = 0.25;
a3 = 0.26;

%%transformatiom matrix A1~A6

A1 = [ cos(theta1)              0    -sin(theta1)   a1*cos(theta1);
       sin(theta1)              0     cos(theta1)   a1*sin(theta1);
                 0             -1              0                 0;
                 0              0              0                 1  ];
             
A2 = [ cos(theta2)   -sin(theta2)              0    a2*cos(theta2);
       sin(theta2)    cos(theta2)              0    a2*sin(theta2);
                 0              0              1                 0;
                 0              0              0                 1  ];

A3 = [ cos(theta3)   -sin(theta3)              0    a3*cos(theta3);
       sin(theta3)    cos(theta3)              0    a3*sin(theta3);
                 0              0              1                 0;
                 0              0              0                 1  ];
             
A4 = [ cos(theta4)              0    -sin(theta4)                0;
       sin(theta4)              0     cos(theta4)                0;
                 0             -1              0                 0;
                 0              0              0                 1  ];
             
A5 = [ cos(theta5)              0     sin(theta5)                0;
       sin(theta5)              0    -cos(theta5)                0;
                 0              1              0                 0;
                 0              0              0                 1  ];
             
A6 = [ cos(theta6)   -sin(theta6)              0                 0;
       sin(theta6)    cos(theta6)              0                 0;
                 0              0              1                 0;
                 0              0              0                 1  ];

T6 = A1*A2*A3*A4*A5*A6;             
T3 = A1*A2*A3;

x = (a1 + a2*cos(theta2) + a3*cos(theta2+theta3))*cos(theta1);
y = (a1 + a2*cos(theta2) + a3*cos(theta2+theta3))*sin(theta1);
z = -a2*sin(theta2) - a3*sin(theta2+theta3);

phi = atan(T6(10)/T6(9));
theta = atan((cos(phi)*T6(9) + sin(phi)*T6(10))/T6(11))+pi;
psi = atan((-sin(phi)*T6(1) + cos(phi)*T6(2))/(-sin(phi)*T6(5) + cos(phi)*T6(6)));

% disp('input:')
% fprintf('\n')
% disp('(£c1,£c1,£c3,£c4,£c5,£c6) =')
% fprintf('\n') 
% disp([theta1/pi*180 theta2/pi*180 theta3/pi*180 theta4/pi*180 theta5/pi*180 theta6/pi*180])
% disp('output:')
% fprintf('\n')
% disp('(n,o,a,p) =')
% disp(T6)
% disp('(x,y,z,£r,£c,£p) =')
% disp([T6(13) T6(14) T6(15 ) phi theta psi])