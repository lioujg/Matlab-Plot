clear all; close all;clc


theta1 = 50;
theta2 = 50;
theta3 = 50;
theta4 = 50;
theta5 = 50;
theta6 = 50;



a2 = 0.432;
a3 = -0.02;

%%transformatiom matrix A1~A6

A1 = [ cosd(theta1)              0   -sind(theta1)                  0;
       sind(theta1)              0    cosd(theta1)                  0;
                  0             -1               0                  0;
                  0              0               0                  1  ];
             
A2 = [ cosd(theta2)   -sind(theta2)              0    a2*cosd(theta2);
       sind(theta2)    cosd(theta2)              0    a2*sind(theta2);
                  0               0              1                  0;
                  0               0              0                  1  ];

A3 = [ cosd(theta3)              0    sind(theta3)    a3*cosd(theta3);
       sind(theta3)              0   -cosd(theta3)    a3*sind(theta3);
                  0              1               0              0.149;
                  0              0               0                  1  ];
             
A4 = [ cosd(theta4)              0    -sind(theta4)                0;
       sind(theta4)              0     cosd(theta4)                0;
                  0             -1                0            0.433;
                  0              0                0                1  ];
             
A5 = [ cosd(theta5)              0     sind(theta5)                0;
       sind(theta5)              0    -cosd(theta5)                0;
                  0              1                0                0;
                  0              0                0                1  ];
             
A6 = [ cosd(theta6)   -sind(theta6)              0                 0;
       sind(theta6)    cosd(theta6)              0                 0;
                  0               0              1                 0;
                  0               0              0                 1  ];

T6 = A1*A2*A3*A4*A5*A6;

x = (a2*cosd(theta2) + a3*cosd(theta2+theta3))*cosd(theta1);
y = (a2*cosd(theta2) + a3*cosd(theta2+theta3))*sind(theta1);
z = -a2*sind(theta2) - a3*sind(theta2+theta3);

phi = atan2d(T6(10),T6(9));
theta = atan2d((cosd(phi)*T6(9) + sind(phi)*T6(10)),T6(11));
psi = atan2d((-sind(phi)*T6(1) + cosd(phi)*T6(2)),(-sind(phi)*T6(5) + cosd(phi)*T6(6)));

disp('input:')
fprintf('\n')
disp('(theta1,theta1,theta3,theta4,theta5,theta6) =')
fprintf('\n') 
disp([theta1 theta2 theta3 theta4 theta5 theta6])
disp('output:')
fprintf('\n')
disp('(n,o,a,p) =')
disp(T6)
disp('(x,y,z,phy,theta,psy) =')
disp([T6(13) T6(14) T6(15) phi theta psi])