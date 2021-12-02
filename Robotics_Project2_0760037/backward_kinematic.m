clear all; close all;clc

%%parameters

% noap = [ -0.2032 -0.6320 -0.7478   0.4072;
%          -0.9485 -0.0624  0.3105   0.2351;
%          -0.2429  0.7724 -0.5868  -0.3465;
%                0       0       0        1];



%noap = [  0.9788  0.0222  0.2036   0.3083;
%         -0.1343 -0.6808  0.7200   0.0826;
%          0.1546 -0.7321 -0.6634  -0.4171;
%               0       0       0        1];   

%noap = [0 0 1 0.1;-1 0 0 0.5;0 -1 0 0.3;0 0 0 1];
% noap = [0 0 1 0.3;0 1 0 0.3;-1 0 0 0.2;0 0 0 1];

%noap = [0 1 0 -0.3;0 0 -1 -0.25;-1 0 0 0.25;0 0 0 1];

A = [0  1  0   0.2;
    -1  0  0   0.3;
     0  0  1   0.2;
     0  0  0  0.01  ];
B = [ 0  0 -1   -0.1;
     -1  0  0   0.15;
      0  1  0    0.3;
      0  0  0   0.01  ];
noap = [ 1  0  0  -0.25;
      0 -1  0    0.1;
      0  0 -1   -0.2;
      0  0  0   0.01  ];

nx = noap(1);
ny = noap(2);
nz = noap(3);
ox = noap(5);
oy = noap(6);
oz = noap(7);
ax = noap(9);
ay = noap(10);
az = noap(11);
px = noap(13);
py = noap(14);
pz = noap(15);

a1 = 0.12;
a2 = 0.25;
a3 = 0.26;

% phi = atan(ay/ax);
phi = atan2(ay,ax);
% if phi<0
%      phi = phi + pi;
% end
% thetaa = atan((cos(phi)*ax + sin(phi)*ay)/az);
thetaa = atan2((cos(phi)*ax + sin(phi)*ay),az);
% if thetaa<0
%    thetaa = thetaa + pi; 
% end    
% psi = atan((-sin(phi)*nx + cos(phi)*ny)/(-sin(phi)*ox + cos(phi)*oy))-pi;
psi = atan2((-sin(phi)*nx + cos(phi)*ny),(-sin(phi)*ox + cos(phi)*oy));
if(psi < 0)
   psi = psi +pi;
end
%% theta1 ~ theta3 solved by geometric solution 

theta1 = atan2(py,px);

gama = atan2(-pz,(px^2+py^2)^(0.5)-a1);
R = (((px^2+py^2)^(0.5)-a1)^2 + pz^2)^(0.5);
aphar = acos((R^2+a2^2-a3^2)/(2*R*a2));

theta2_1 = (gama - aphar);
theta2_2 = (gama + aphar);

beta = acos((R^2+a3^2-a2^2)/(2*R*a3));
theta3_1 = aphar + beta;
theta3_2 = -aphar - beta;

%% theta4 ~ theta6 solved by algebraic solution 

tan_theta4_1 = (-cos(theta1)*sin(theta2_1+theta3_1)*ax - sin(theta1)*sin(theta2_1+theta3_1)*ay - cos(theta2_1+theta3_1)*az) / (cos(theta1)*cos(theta2_1+theta3_1)*ax + sin(theta1)*cos(theta2_1+theta3_1)*ay - sin(theta2_1+theta3_1)*az);
theta4_1_1 = atan(tan_theta4_1);
if(theta4_1_1<0)
   theta4_1_2 = theta4_1_1+pi;
else
   theta4_1_2 = theta4_1_1-pi; 
end
tan_theta4_2 = (-cos(theta1)*sin(theta2_2+theta3_2)*ax - sin(theta1)*sin(theta2_2+theta3_2)*ay - cos(theta2_2+theta3_2)*az) / (cos(theta1)*cos(theta2_2+theta3_2)*ax + sin(theta1)*cos(theta2_2+theta3_2)*ay - sin(theta2_2+theta3_2)*az);
theta4_2_1 = atan(tan_theta4_2);
if(theta4_2_1<0)
   theta4_2_2 = theta4_2_1+pi;
else
   theta4_2_2 = theta4_2_1-pi; 
end

theta5_1 = atan2(ax*cos(theta1)*cos(theta2_1+theta3_1+theta4_1_1) + ay*sin(theta1)*cos(theta2_1+theta3_1+theta4_1_1) - az*sin(theta2_1+theta3_1+theta4_1_1) , -ax*sin(theta1)+ay*cos(theta1));
theta5_2 = atan2(ax*cos(theta1)*cos(theta2_1+theta3_1+theta4_1_2) + ay*sin(theta1)*cos(theta2_1+theta3_1+theta4_1_2) - az*sin(theta2_1+theta3_1+theta4_1_2) , -ax*sin(theta1)+ay*cos(theta1));
theta5_3 = atan2(ax*cos(theta1)*cos(theta2_2+theta3_2+theta4_2_1) + ay*sin(theta1)*cos(theta2_2+theta3_2+theta4_2_1) - az*sin(theta2_2+theta3_2+theta4_2_1) , -ax*sin(theta1)+ay*cos(theta1));
theta5_4 = atan2(ax*cos(theta1)*cos(theta2_2+theta3_2+theta4_2_2) + ay*sin(theta1)*cos(theta2_2+theta3_2+theta4_2_2) - az*sin(theta2_2+theta3_2+theta4_2_2) , -ax*sin(theta1)+ay*cos(theta1));

theta6_1 = atan2(-nx*cos(theta1)*sin(theta2_1+theta3_1+theta4_1_1)-ny*sin(theta1)*sin(theta2_1+theta3_1+theta4_1_1)-nz*cos(theta2_1+theta3_1+theta4_1_1) , -ox*cos(theta1)*sin(theta2_1+theta3_1+theta4_1_1)-oy*sin(theta1)*sin(theta2_1+theta3_1+theta4_1_1)-oz*cos(theta2_1+theta3_1+theta4_1_1));
theta6_2 = atan2(-nx*cos(theta1)*sin(theta2_1+theta3_1+theta4_1_2)-ny*sin(theta1)*sin(theta2_1+theta3_1+theta4_1_2)-nz*cos(theta2_1+theta3_1+theta4_1_2) , -ox*cos(theta1)*sin(theta2_1+theta3_1+theta4_1_2)-oy*sin(theta1)*sin(theta2_1+theta3_1+theta4_1_2)-oz*cos(theta2_1+theta3_1+theta4_1_2));
theta6_3 = atan2(-nx*cos(theta1)*sin(theta2_2+theta3_2+theta4_2_1)-ny*sin(theta1)*sin(theta2_2+theta3_2+theta4_2_1)-nz*cos(theta2_2+theta3_2+theta4_2_1) , -ox*cos(theta1)*sin(theta2_2+theta3_2+theta4_2_1)-oy*sin(theta1)*sin(theta2_2+theta3_2+theta4_2_1)-oz*cos(theta2_2+theta3_2+theta4_2_1));
theta6_4 = atan2(-nx*cos(theta1)*sin(theta2_2+theta3_2+theta4_2_2)-ny*sin(theta1)*sin(theta2_2+theta3_2+theta4_2_2)-nz*cos(theta2_2+theta3_2+theta4_2_2) , -ox*cos(theta1)*sin(theta2_2+theta3_2+theta4_2_2)-oy*sin(theta1)*sin(theta2_2+theta3_2+theta4_2_2)-oz*cos(theta2_2+theta3_2+theta4_2_2));

theta = [theta1/pi*180 theta2_1/pi*180 theta3_1/pi*180 theta4_1_1/pi*180 theta5_1/pi*180 theta6_1/pi*180;
         theta1/pi*180 theta2_1/pi*180 theta3_1/pi*180 theta4_1_2/pi*180 theta5_2/pi*180 theta6_2/pi*180;
         theta1/pi*180 theta2_2/pi*180 theta3_2/pi*180 theta4_2_1/pi*180 theta5_3/pi*180 theta6_3/pi*180;
         theta1/pi*180 theta2_2/pi*180 theta3_2/pi*180 theta4_2_2/pi*180 theta5_4/pi*180 theta6_4/pi*180];


disp('input:')
fprintf('\n')
disp('(n,o,a,p) =')
disp(noap)
disp('(x,y,z,?r,?c,?p) =')
disp([px py pz phi thetaa psi])
disp('output:')
fprintf('\n')
disp('(?c1,?c1,?c3,?c4,?c5,?c6) =')
fprintf('----------------------------------------------------\n') ;
for i = 1:4
    fprintf('Ans%d:\n', i) ;
    disp(theta(i,:))
    if theta(i,1) < -150 || theta(i,1) > 150
       fprintf('theta1 is out of range\n') ; 
    end    
    if theta(i,2) < -30 || theta(i,2) > 100
       fprintf('theta2 is out of range\n') ; 
    end 
    if theta(i,3) < -120 || theta(i,3) > 0
       fprintf('theta3 is out of range\n') ; 
    end 
    if theta(i,4) < -110 || theta(i,4) > 110
       fprintf('theta4 is out of range\n') ; 
    end 
    if theta(i,5) < -180 || theta(i,5) > 180
       fprintf('theta5 is out of range\n') ; 
    end 
    if theta(i,6) < -180 || theta(i,6) > 180
       fprintf('theta6 is out of range\n') ; 
    end 
    fprintf('----------------------------------------------------\n') ;
end