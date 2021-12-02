function [A ,B] = Kinematics(theta)
%================================= input ================================
%Theta = input('Input theta 1 to theta 6 = [theta1 theta2 theta3 theta4 theta4 theta5 theta6]\n');
Theta = theta;
%================================== test mode =============================

%Theta = [         0    0.0000   -0.0000         0         0         0 ];
%Theta = [         0    0.0000   -0.0000         0         0         0 ];
%Theta = [         0  267.1527 -174.7108         0         0         0 ];
%Theta = [         0  267.1527 -174.7108         0         0         0 ];
%Theta = [ -140.2349  -87.1527   -0.0000         0         0         0 ];
%Theta = [ -140.2349  -87.1527   -0.0000         0         0         0 ];
%Theta = [ -140.2349  180.0000 -174.7108         0         0         0 ];
%Theta = [ -140.2349  180.0000 -174.7108         0         0         0 ] ;

%======compute the cos(theta1),sin(theta1) to cos(theta6),sin(theta6)======

for i = 1:1:6
   c(i) = cos(pi/180*Theta(i));
   s(i) = sin(pi/180*Theta(i));
end

%================================== kinetic tbale =========================
d3 = 0.149; d4 = 0.433; a2 = 0.432; a3 =-0.02;

A1 = [c(1) 0 -s(1) 0; s(1) 0 c(1) 0; 0 -1 0 0; 0 0 0 1];
A2 = [c(2) -s(2) 0 a2*c(2); s(2) c(2) 0 a2*s(2); 0 0 1 0; 0 0 0 1];
A3 = [c(3) 0 s(3) a3*c(3); s(3) 0 -c(3) a3*s(3); 0 1 0 d3; 0 0 0 1];
A4 = [c(4) 0 -s(4) 0; s(4) 0 c(4) 0; 0 -1 0 d4; 0 0 0 1];
A5 = [c(5) 0 s(5) 0; s(5) 0 -c(5) 0; 0 1 0 0; 0 0 0 1];
A6 = [c(6) -s(6) 0 0; s(6) c(6) 0 0; 0 0 1 0; 0 0 0 1];

T6 = A1*A2*A3*A4*A5*A6;

%====================compute the x , y , z , phi , theta , phi ============

x = T6(1,4);
y = T6(2,4);
z = T6(3,4);

phi1 = 180/pi*atan2(T6(2,3),T6(1,3));

THy = T6(1,3)*cos(pi/180*phi1) + T6(2,3)*sin(pi/180*phi1);
THx = T6(3,3);s

theta = atan2(THy,THx);

phi2y = -T6(1,1)*sin(pi/180*phi1) + T6(2,1)*cos(pi/180*phi1);
phi2x = -T6(1,2)*sin(pi/180*phi1) + T6(2,2)*cos(pi/180*phi1);

phi2 = atan2(phi2y,phi2x);
%================================== show the resilt =======================
%fprintf('x         y         z         phi         theta         phi\n');
%fprintf('%f  %f  %f  %f  %f  %f',x,y,z,phi1,theta,phi2);
A = [x y z phi1 theta phi2];
B = T6;