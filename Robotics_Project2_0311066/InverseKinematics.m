function A = InverseKinematics(noap)
% this function is modif from midterm project
%%================================= input =================================
%{
N = input('Input array n = [nx ny nz]\n');
O = input('Input array O = [ox oy oz]\n');
A = input('Input array A = [ax ay az]\n');
P = input('Input array P = [px py pz]\n');

%================================== test mode =============================

%N = [1 0 0];
%O = [0 1 0];
%A = [0 0 1];
%P = [0.412 0.149 0.433];
%}
%================================== cartesian point =======================
%{
nx = N(1);ny = N(2);nz = N(3);                 % define n
ox = O(1);oy = O(2);oz = O(3);                 % define o
ax = A(1);ay = A(2);az = A(3);                 % define a 
px = P(1);py = P(2);pz = P(3);                 % define p
%}
nx = noap(1,1);ny = noap(2,1);nz = noap(3,1);  % define n
ox = noap(1,2);oy = noap(2,2);oz = noap(3,2);  % define o
ax = noap(1,3);ay = noap(2,3);az = noap(3,3);  % define a
px = noap(1,4);py = noap(2,4);pz = noap(3,4);  % define p
Cartesian_Point = [nx ox ax px; ny oy ay py; nz oz az pz; 0 0 0 1];

%================================== kinetic tbale =========================

answer = zeros(8,6);
d3 = 0.149; d4 = 0.433; a2 = 0.432; a3 =-0.02;
%{
A1 = [c1 0 -s1 0; s1 0 c1 0; 0 -1 0 0; 0 0 0 1];
A2 = [c2 -s2 0 a2*c2; s2 c2 0 a2*s2; 0 0 1 0; 0 0 0 1];
A3 = [c3 0 s3 a3*c3; s3 0 -c3 a3*s3; 0 1 0 d3; 0 0 0 1];
A4 = [c4 0 -s4 0; s4 0 c4 0; 0 -1 0 d4; 0 0 0 1];c
A5 = [c5 0 s5 0; s5 0 -c5 0; 0 1 0 0; 0 0 0 1];
A6 = [c6 -s6 0 0; s6 c6 0 0; 0 0 1 0; 0 0 0 1];
%}

%================================= solve theta 1 ==========================

theta_1 = zeros(1,8);
FiTheta = zeros(1,8);
FiTheta(1:4) = 180/pi*atan2(d3,((px^2 + py^2 - d3^2)^0.5));
FiTheta(5:8) = 180/pi*atan2(d3,(-(px^2 + py^2 - d3^2)^0.5));
for i = 1:1:8
    theta_1(i) = 180/pi*atan2(py,px)-FiTheta(i);
    answer(i,1) = theta_1(i);
end

%================================= solve theta 3 ==========================

M = (px^2 + py^2 + pz^2 -a2^2 - a3^2 -d3^2 -d4^2)/(2*a2);
theta_3 = zeros(1,8);
theta3_support = zeros(1,2);
theta3_support(1) = sqrt(a3^2+d4^2-M^2);
theta3_support(2) = -sqrt(a3^2+d4^2-M^2);

for i = 1:1:2
    for j =1:1:2
    theta_3(2*i-2+j) = rad2deg(atan2(M,theta3_support(i))-atan2(a3,d4));
    theta_3(2*i+2+j) = rad2deg(atan2(M,theta3_support(i))-atan2(a3,d4));
    end
end
answer(1:8,3) = theta_3(1:8);

%================================= solve theta 2 ==========================

% m1*x - n1*y = z1
% m2*x + n2*y = z2 
%-> solve that x = (z1*n2 + n1*z2)/(m1*n2 + m2*n1)
%-> y = (z2*m1 - z1*m2)/(n2*m1 + n1*m2)
theta_2 = zeros(1,8);
for i = 1:1:8
       m1 = px*cos(pi/180*theta_1(i)) + py*sin(pi/180*theta_1(i));
       m2 = pz;
       n1 = pz; 
       n2 = m1;
       z1 = a3 + a2*cos(pi/180*theta_3(i));
       z2 = d4 + a2*sin(pi/180*theta_3(i));
       x = (z1*n2 + z2*n1)/(m1*n2 + m2*n1);
       y = (z2*m1 - z1*m2)/(n2*m1 + n1*m2);
       theta_2(i) = (180/pi)*atan2(y,x) - theta_3(i);
end
answer(1:8,2) = theta_2(1:8);

%================================= solve theta 4 ==========================

theta_4 = zeros(1,8);
for i = 1:2:7
        c1 = cos(pi/180*theta_1(i));
        s1 = sin(pi/180*theta_1(i));
        c2 = cos(pi/180*theta_2(i));
        s2 = sin(pi/180*theta_2(i));
        c3 = cos(pi/180*theta_3(i));
        s3 = sin(pi/180*theta_3(i));
        A1 = [c1 0 -s1 0; s1 0 c1 0; 0 -1 0 0; 0 0 0 1];
        A2 = [c2 -s2 0 a2*c2; s2 c2 0 a2*s2; 0 0 1 0; 0 0 0 1];
        A3 = [c3 0 s3 a3*c3; s3 0 -c3 a3*s3; 0 1 0 d3; 0 0 0 1];
        T36 = inv(A3)*inv(A2)*inv(A1)*Cartesian_Point;
        theta_4(i) = 180/pi*atan2(T36(2,3),T36(1,3));
end
for i = 2:2:8
        c1 = cos(pi/180*theta_1(i));
        s1 = sin(pi/180*theta_1(i));
        c2 = cos(pi/180*theta_2(i));
        s2 = sin(pi/180*theta_2(i));
        c3 = cos(pi/180*theta_3(i));
        s3 = sin(pi/180*theta_3(i));
        A1 = [c1 0 -s1 0; s1 0 c1 0; 0 -1 0 0; 0 0 0 1];
        A2 = [c2 -s2 0 a2*c2; s2 c2 0 a2*s2; 0 0 1 0; 0 0 0 1];
        A3 = [c3 0 s3 a3*c3; s3 0 -c3 a3*s3; 0 1 0 d3; 0 0 0 1];
        T36 = inv(A3)*inv(A2)*inv(A1)*Cartesian_Point;
        theta_4(i) = 180/pi*atan2(-T36(2,3),-T36(1,3));
end
answer(1:8,4) = theta_4(1:8);

%================================= solve theta 5 ==========================

theta_5 = zeros(1,8);
for i = 1:1:8
        c1 = cos(pi/180*theta_1(i));
        s1 = sin(pi/180*theta_1(i));
        c2 = cos(pi/180*theta_2(i));
        s2 = sin(pi/180*theta_2(i));
        c3 = cos(pi/180*theta_3(i));
        s3 = sin(pi/180*theta_3(i));
        c4 = cos(pi/180*theta_4(i));
        s4 = sin(pi/180*theta_4(i));
        A1 = [c1 0 -s1 0; s1 0 c1 0; 0 -1 0 0; 0 0 0 1];
        A2 = [c2 -s2 0 a2*c2; s2 c2 0 a2*s2; 0 0 1 0; 0 0 0 1];
        A3 = [c3 0 s3 a3*c3; s3 0 -c3 a3*s3; 0 1 0 d3; 0 0 0 1];
        A4 = [c4 0 -s4 0; s4 0 c4 0; 0 -1 0 d4; 0 0 0 1];
        T46 = inv(A4)*inv(A3)*inv(A2)*inv(A1)*Cartesian_Point;
        theta_5(i) = 180/pi*atan2(T46(1,3),-T46(2,3));
end
answer(1:8,5) = theta_5(1:8);

%================================= solve theta 6 ==========================

theta_6 = zeros(1,8);
for i = 1:1:8
        c1 = cos(pi/180*theta_1(i));
        s1 = sin(pi/180*theta_1(i));
        c2 = cos(pi/180*theta_2(i));
        s2 = sin(pi/180*theta_2(i));
        c3 = cos(pi/180*theta_3(i));
        s3 = sin(pi/180*theta_3(i));
        A1 = [c1 0 -s1 0; s1 0 c1 0; 0 -1 0 0; 0 0 0 1];
        A2 = [c2 -s2 0 a2*c2; s2 c2 0 a2*s2; 0 0 1 0; 0 0 0 1];
        A3 = [c3 0 s3 a3*c3; s3 0 -c3 a3*s3; 0 1 0 d3; 0 0 0 1];
        T36 = inv(A3)*inv(A2)*inv(A1)*Cartesian_Point;
        theta_6(i) = 180/pi*atan2(T36(3,2)/theta_5(i),-T36(3,1)/theta_5(i));
end
answer(1:8,6) = theta_6(1:8);
A = answer;

%================================= print the answer =======================
%{
 fprintf('        theta1     theta2     theta3     theta4     theta5     theta6\n',i);
for i = 1:1:8
   fprintf('\n\nAnswer(%d)\n',i);   
   fprintf('       %f   %f   %f   %f   %f   %f\n\n',theta_1(i), theta_2(i), theta_3(i), theta_4(i), theta_5(i), theta_6(i));
   if 160 > theta_1(i) > -160
   else
      fprintf('theta 1 is out of range\n'); 
   end
   if theta_2(i) > 125 || theta_2(i) < -125
      fprintf('theta 2 is out of range\n'); 
   end
   if theta_3(i) > 135 || theta_3(i) < -133
      fprintf('theta 3 is out of range\n'); 
   end
   if theta_4(i) > 140 || theta_4(i) < -140
      fprintf('theta 4 is out of range\n'); 
   end
   if theta_5(i) > 100 || theta_5(i) < -100
      fprintf('theta 5 is out of range\n'); 
   end
   if theta_6(i) > -260 || theta_6(i) < -260
      fprintf('theta 6 is out of range\n'); 
   end   
end
%}