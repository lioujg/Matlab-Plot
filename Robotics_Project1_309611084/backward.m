clear
clc
%%================================= input =================================

N = input('Input array n = [nx ny nz]\n');
O = input('Input array O = [ox oy oz]\n');
A = input('Input array A = [ax ay az]\n');
P = input('Input array P = [px py pz]\n');

%================================== cartesian point =======================

nx = N(1);ny = N(2);nz = N(3);                 
ox = O(1);oy = O(2);oz = O(3);                 
ax = A(1);ay = A(2);az = A(3);                
px = P(1);py = P(2);pz = P(3);                 
C_P = [nx ox ax px;
       ny oy ay py;
       nz oz az pz;
        0  0  0  1  ];

%================================== kinetic tbale =========================

ans = zeros(8,6);
d3 = 0.149; d4 = 0.433; a2 = 0.432; a3 =-0.02;

%{
A1 = [c1  0 -s1  0;
      s1  0  c1  0;
       0 -1   0  0;
       0  0   0  1  ];
A2 = [c2 -s2  0 a2*c2;
      s2  c2  0 a2*s2;
       0   0  1     0;
       0   0  0     1  ];
A3 = [c3  0  s3 a3*c3;
      s3  0 -c3 a3*s3;
       0  1   0    d3;
       0  0   0     1  ];
A4 = [c4  0 -s4  0;
      s4  0  c4  0;
       0 -1   0 d4;
       0  0   0  1  ];
A5 = [c5  0  s5  0;
      s5  0 -c5  0;
       0  1   0  0;
       0  0   0  1  ];
A6 = [c6 -s6  0  0;
      s6  c6  0  0;
       0   0  1  0;
       0   0  0  1  ];
%}

%================================= solve theta 1 ==========================

theta_1 = zeros(1,8);
FiTheta = zeros(1,8);
FiTheta(1:4) = atan2d(d3,((px^2 + py^2 - d3^2)^0.5));
FiTheta(5:8) = atan2d(d3,(-(px^2 + py^2 - d3^2)^0.5));
for i = 1:1:8
    theta_1(i) = atan2d(py,px)-FiTheta(i);
    ans(i,1) = theta_1(i);
end

%================================= solve theta 3 ==========================

M = (px^2 + py^2 + pz^2 -a2^2 - a3^2 -d3^2 -d4^2)/(2*a2);
theta_3 = zeros(1,8);
theta_3s = zeros(1,2);
theta_3s(1) = sqrt(a3^2+d4^2-M^2);
theta_3s(2) = -sqrt(a3^2+d4^2-M^2);

for i = 1:1:2
    for j =1:1:2
    theta_3(2*i-2+j) = rad2deg(atan2(M,theta_3s(i))-atan2(a3,d4));
    theta_3(2*i+2+j) = rad2deg(atan2(M,theta_3s(i))-atan2(a3,d4));
    end
end
ans(1:8,3) = theta_3(1:8);

%================================= solve theta 2 ==========================

theta_2 = zeros(1,8);
for i = 1:1:8
       m1 = px*cosd(theta_1(i)) + py*sind(theta_1(i));
       m2 = pz;
       n1 = pz; 
       n2 = m1;
       z1 = a3 + a2*cosd(theta_3(i));
       z2 = d4 + a2*sind(theta_3(i));
       x = (z1*n2 + z2*n1)/(m1*n2 + m2*n1);
       y = (z2*m1 - z1*m2)/(n2*m1 + n1*m2);
       theta_2(i) = atan2d(y,x) - theta_3(i);
end
ans(1:8,2) = theta_2(1:8);

%================================= solve theta 4 ==========================

theta_4 = zeros(1,8);
for i = 1:2:7   %2,4,6,8 are 0
        c1 = cosd(theta_1(i));
        c2 = cosd(theta_2(i));
        c3 = cosd(theta_3(i));        
        s1 = sind(theta_1(i));        
        s2 = sind(theta_2(i));        
        s3 = sind(theta_3(i));
        
        A1 = [c1  0 -s1  0;
              s1  0  c1  0;
               0 -1   0  0;
               0  0   0  1  ];
        A2 = [c2 -s2  0 a2*c2;
              s2  c2  0 a2*s2;
               0   0  1     0;
               0   0  0     1  ];
        A3 = [c3 0  s3 a3*c3;
              s3 0 -c3 a3*s3; 
               0 1   0    d3;
               0 0   0     1  ];
        T3_6 = inv(A3)*inv(A2)*inv(A1)*C_P;
        theta_4(i) = atan2d(T3_6(2,3),T3_6(1,3));
end
for i = 2:2:8   %decide 2,4,6,8
        c1 = cosd(theta_1(i));
        c2 = cosd(theta_2(i));
        c3 = cosd(theta_3(i));        
        s1 = sind(theta_1(i));        
        s2 = sind(theta_2(i));        
        s3 = sind(theta_3(i));
        
        A1 = [c1  0 -s1 0;
              s1  0  c1 0;
               0 -1   0 0;
               0  0   0 1  ];
        A2 = [c2 -s2  0 a2*c2;
              s2  c2  0 a2*s2;
               0   0  1     0;
               0   0  0     1  ];
        A3 = [c3 0  s3 a3*c3;
              s3 0 -c3 a3*s3;
               0 1   0    d3; 
               0 0   0     1  ];
        T3_6 = inv(A3)*inv(A2)*inv(A1)*C_P;
        theta_4(i) = atan2d(-T3_6(2,3),-T3_6(1,3)); %minus
end
ans(1:8,4) = theta_4(1:8);

%================================= solve theta 5 ==========================

theta_5 = zeros(1,8);
for i = 1:1:8
        c1 = cosd(theta_1(i));
        c2 = cosd(theta_2(i));
        c3 = cosd(theta_3(i));
        c4 = cosd(theta_4(i));
        s1 = sind(theta_1(i));        
        s2 = sind(theta_2(i));        
        s3 = sind(theta_3(i));        
        s4 = sind(theta_4(i));
        
        A1 = [c1  0 -s1 0;
              s1  0  c1 0;
               0 -1   0 0;
               0  0   0 1  ];
        A2 = [c2 -s2  0 a2*c2;
              s2  c2  0 a2*s2;
               0   0  1     0;
               0   0  0     1];
        A3 = [c3 0  s3 a3*c3;
              s3 0 -c3 a3*s3;
               0 1   0    d3;
               0 0   0     1  ];
        A4 = [c4  0 -s4   0;
              s4  0  c4   0;
               0 -1   0  d4;
               0  0   0   1  ];
        T46 = inv(A4)*inv(A3)*inv(A2)*inv(A1)*C_P;
        theta_5(i) = atan2d(T46(1,3),-T46(2,3));
end
ans(1:8,5) = theta_5(1:8);

%================================= solve theta 6 ==========================

theta_6 = zeros(1,8);
for i = 1:1:8
        c1 = cosd(theta_1(i));
        c2 = cosd(theta_2(i));
        c3 = cosd(theta_3(i));
        s1 = sind(theta_1(i));        
        s2 = sind(theta_2(i));        
        s3 = sind(theta_3(i));
        A1 = [c1  0 -s1 0;
              s1  0  c1 0;
               0 -1   0 0;
               0  0   0 1  ];
        A2 = [c2 -s2 0 a2*c2;
              s2  c2 0 a2*s2;
               0   0 1     0; 
               0   0 0     1  ];
        A3 = [c3 0  s3 a3*c3;
              s3 0 -c3 a3*s3; 
               0 1   0    d3;
               0 0   0     1  ];
        T3_6 = inv(A3)*inv(A2)*inv(A1)*C_P;
        theta_6(i) = atan2d(T3_6(3,2)/sin(theta_5(i)),-T3_6(3,1)/sin(theta_5(i)));
end
ans(1:8,6) = theta_6(1:8);
ans();

%================================= print the answer =======================

 fprintf('        theta1      theta2      theta3      theta4      theta5      theta6\n',i);
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
   
   if theta_3(i) > 135 || theta_3(i) < -135
      fprintf('theta 3 is out of range\n'); 
   end
   if theta_4(i) > 140 || theta_4(i) < -140
      fprintf('theta 4 is out of range\n'); 
   end
   if theta_5(i) > 100 || theta_5(i) < -100
      fprintf('theta 5 is out of range\n'); 
   end
   if theta_6(i) > 260 || theta_6(i) < -260
      fprintf('theta 6 is out of range\n'); 
   end   
end