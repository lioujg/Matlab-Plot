clc
clear

%A B C 's matrix
A = [0 0 1 -0.30; -1 0 0 0.20; 0 -1 0 0.30; 0 0 0 1];
B = [0 0 1 0.50; 0 1 0 -0.20; -1 0 0 0.60; 0 0 0 1];
C = [-1 0 0 -0.25; 0 0 1 0.20; 0 1 0 -0.30; 0 0 0 1];

% define the n o a p in A B C
nax = A(1,1);nay = A(2,1);naz = A(3,1);
oax = A(1,2);oay = A(2,2);oaz = A(3,2);
aax = A(1,3);aay = A(2,3);aaz = A(3,3);
pax = A(1,4);pay = A(2,4);paz = A(3,4);

nbx = B(1,1);nby = B(2,1);nbz = B(3,1);
obx = B(1,2);oby = B(2,2);obz = B(3,2);
abx = B(1,3);aby = B(2,3);abz = B(3,3);
pbx = B(1,4);pby = B(2,4);pbz = B(3,4);

ncx = C(1,1);ncy = C(2,1);ncz = C(3,1);
ocx = C(1,2);ocy = C(2,2);ocz = C(3,2);
acx = C(1,3);acy = C(2,3);acz = C(3,3);
pcx = C(1,4);pcy = C(2,4);pcz = C(3,4);

%Use the InverseKinematic to solve theta1~theta6 which this function is
%from the midterm project
Theta_A = InverseKinematics(A);
Theta_B = InverseKinematics(B);
Theta_C = InverseKinematics(C);

%because the theta1~6 from IK are 8 result so we choose the first result
%here
result = 1;
joint_A1 = Theta_A(result,:);
joint_B1 = Theta_B(result,:);
joint_C1 = Theta_C(result,:);

sampling = 0.002;
tacc = 0.2;
T = 0.5;
%find A' poiint from linear method and delta B &C
joint_A2 = joint_A1 + (joint_B1 - joint_A1)/T*(T-tacc);
delta_B = joint_A1 - joint_B1;
delta_C = joint_C1 - joint_B1;

%% the Joint define & path
%define tacc and period time
s=1;

%find the theta 、velocity of theta 、acceleration of theta from A to A' 
for t = (-T):(sampling):(-tacc)
    h = t/T;
    joint_A(:,s) = joint_B1 - delta_B/0.5*t;
    djoint_A(:,s) = -delta_B/T;
    ddjoint_A(:,s) = [0;0;0;0;0;0];
    s = s+1;
end

%find the theta 、velocity of theta 、acceleration of theta due to the
%transition method
s=1;
for t = (-tacc+sampling):(sampling):(tacc-sampling)
    h = (t+tacc)/(2*tacc);
    joint_B(:,s)= (tacc/T)*((delta_C+delta_B)*(2-h)*(h.^2)-2*delta_B)*h+delta_B*(tacc/T)+joint_B1;
    djoint_B(:,s) = ((delta_C+delta_B)*(1.5-h)*2*(h.^2)-delta_B)/0.5;
    ddjoint_B(:,s) = (delta_C+delta_B)*(1-h)*3*h/(tacc*T);
    s = s+1;
end

%find the theta 、velocity of theta 、acceleration of theta to the final
%position C
s=1;
for t = tacc:sampling:T
    h = t/T;
    joint_C(:,s) = delta_C*h+joint_B1;                                
    djoint_C(:,s) = delta_C/T;
    ddjoint_C(:,s) = [0;0;0;0;0;0];
    s = s+1;
end


%% plot the result of joint
figure(1)
t = -T:sampling:T;
%combine the result 1 from 3 path 
i=1;
theta1 = [joint_A(i,:) joint_B(i,:) joint_C(i,:)];
dtheta1 = [djoint_A(i,:) djoint_B(i,:) djoint_C(i,:)];
ddtheta1 = [ddjoint_A(i,:) ddjoint_B(i,:) ddjoint_C(i,:)];

subplot(6,3,1);
plot(t,theta1);
grid on
title('Joint Value');
ylabel('theta 1 ');
subplot(6,3,2);
plot(t,dtheta1);
grid on
title('Velocity');
subplot(6,3,3);
plot(t,ddtheta1);
grid on
title('Acceleration');
%combine the result 2 from 3 path 
i=2;
theta2 = [joint_A(i,:) joint_B(i,:) joint_C(i,:)];
dtheta2 = [djoint_A(i,:) djoint_B(i,:) djoint_C(i,:)];
ddtheta2 = [ddjoint_A(i,:) ddjoint_B(i,:) ddjoint_C(i,:)];

subplot(6,3,4);
plot(t,theta2);
grid on
ylabel('theta 2 ');
subplot(6,3,5);
plot(t,dtheta2);
grid on
subplot(6,3,6);
plot(t,ddtheta2);
grid on
%combine the result 3 from 3 path 
i=3;
theta3 = [joint_A(i,:) joint_B(i,:) joint_C(i,:)];
dtheta3 = [djoint_A(i,:) djoint_B(i,:) djoint_C(i,:)];
ddtheta3 = [ddjoint_A(i,:) ddjoint_B(i,:) ddjoint_C(i,:)];

subplot(6,3,7);
plot(t,theta3);
grid on
ylabel('theta 3 ');
subplot(6,3,8);
plot(t,dtheta3);
grid on
subplot(6,3,9);
plot(t,ddtheta3);
grid on
%combine the result 4 from 3 path 
i=4;
theta4 = [joint_A(i,:) joint_B(i,:) joint_C(i,:)];
dtheta4 = [djoint_A(i,:) djoint_B(i,:) djoint_C(i,:)];
ddtheta4 = [ddjoint_A(i,:) ddjoint_B(i,:) ddjoint_C(i,:)];

subplot(6,3,10);
plot(t,theta4);
grid on
ylabel('theta 4 ');
subplot(6,3,11);
plot(t,dtheta4);
grid on
subplot(6,3,12);
plot(t,ddtheta4);
grid on

%combine the result 5 from 3 path 
i=5;
theta5 = [joint_A(i,:) joint_B(i,:) joint_C(i,:)];
dtheta5 = [djoint_A(i,:) djoint_B(i,:) djoint_C(i,:)];
ddtheta5 = [ddjoint_A(i,:) ddjoint_B(i,:) ddjoint_C(i,:)];

subplot(6,3,13);
plot(t,theta5);
grid on
ylabel('theta 5 ');
subplot(6,3,14);
plot(t,dtheta5);
grid on
subplot(6,3,15);
plot(t,ddtheta5);
grid on

%combine the result 6 from 3 path 
i=6;
theta6 = [joint_A(i,:) joint_B(i,:) joint_C(i,:)];
dtheta6 = [djoint_A(i,:) djoint_B(i,:) djoint_C(i,:)];
ddtheta6 = [ddjoint_A(i,:) ddjoint_B(i,:) ddjoint_C(i,:)];

subplot(6,3,16);
plot(t,theta6);
grid on
ylabel('theta 6 ');
subplot(6,3,17);
plot(t,dtheta6);
grid on
subplot(6,3,18);
plot(t,ddtheta6);
grid on

%% plot the result of path
%轉成noap形式畫出軌跡圖
s=1;
for t1 = -T:sampling:-tacc
    [A,B1(:,:,s)] = Kinematics(joint_A(:,s)');
    p1(:,s) = A;
    s = s+1;
end

s=1;
for t2 = (-tacc+sampling):(sampling):(tacc-sampling)
    [A,B2(:,:,s)] = Kinematics(joint_B(:,s)');
    p2(:,s) = A;
    s = s+1;
end

s=1;
for t3 = tacc:sampling:T
    [A,B3(:,:,s)] = Kinematics(joint_C(:,s)');
    p3(:,s) = A;
    s = s+1;
end
X = [p1(1,:) p2(1,:) p3(1,:)];
Y = [p1(2,:) p2(2,:) p3(2,:)];
Z = [p1(3,:) p2(3,:) p3(3,:)];

%劃出軌跡圖
figure(2)
subplot(1,2,1);
plot3(X,Y,Z);
grid on
l = 0.2;
hold on;
plot3([pax,pax+nax*l],[pay,pay+nay*l],[paz,paz+naz*l]);
hold on;
plot3([pax,pax+oax*l],[pay,pay+oay*l],[paz,paz+oaz*l]);
hold on;
plot3([pax,pax+aax*l],[pay,pay+aay*l],[paz,paz+aaz*l]);

hold on;
plot3([pbx,pbx+nbx*l],[pby,pby+nby*l],[pbz,pbz+nbz*l]);
hold on;
plot3([pbx,pbx+obx*l],[pby,pby+oby*l],[pbz,pbz+obz*l]);
hold on;
plot3([pbx,pbx+abx*l],[pby,pby+aby*l],[pbz,pbz+abz*l]);

hold on;
plot3([pcx,pcx+ncx*l],[pcy,pcy+ncy*l],[pcz,pcz+ncz*l]);
hold on;
plot3([pcx,pcx+ocx*l],[pcy,pcy+ocy*l],[pcz,pcz+ocz*l]);
hold on;
plot3([pcx,pcx+acx*l],[pcy,pcy+acy*l],[pcz,pcz+acz*l]);

xlabel('x(m)');
ylabel('y(m)');
zlabel('z(m)');
text(-0.3,0.2,0.3,'A(-0.3,0.2,0.3)');
text(X(151),Y(151),Z(151),'A’');
text(0.5,-0.2,0.6,'B(0.5,-0.2,0.6)');
text(X(1),Y(1),Z(1),'A’');
text(-0.25,0.2,-0.3,'C(-0.25,0.2,-0.3)');
title('3D path of Joint Motion')

subplot(1,2,2);
plot3(X,Y,Z);
grid on
%畫出A點的方向
hold on;
plot3([pax,pax+nax*l],[pay,pay+nay*l],[paz,paz+naz*l]);
hold on;
plot3([pax,pax+oax*l],[pay,pay+oay*l],[paz,paz+oaz*l]);
hold on;
plot3([pax,pax+aax*l],[pay,pay+aay*l],[paz,paz+aaz*l]);
%畫出B點方向
hold on;
plot3([pbx,pbx+nbx*l],[pby,pby+nby*l],[pbz,pbz+nbz*l]);
hold on;
plot3([pbx,pbx+obx*l],[pby,pby+oby*l],[pbz,pbz+obz*l]);
hold on;
plot3([pbx,pbx+abx*l],[pby,pby+aby*l],[pbz,pbz+abz*l]);
%畫出C點方向
hold on;
plot3([pcx,pcx+ncx*l],[pcy,pcy+ncy*l],[pcz,pcz+ncz*l]);
hold on;
plot3([pcx,pcx+ocx*l],[pcy,pcy+ocy*l],[pcz,pcz+ocz*l]);
hold on;
plot3([pcx,pcx+acx*l],[pcy,pcy+acy*l],[pcz,pcz+acz*l]);

xlabel('x(m)');
ylabel('y(m)');
zlabel('z(m)');
text(-0.3,0.2,0.3,'A(-0.3,0.2,0.3)');
text(X(151),Y(151),Z(151),'A’');
text(0.5,-0.2,0.6,'B(0.5,-0.2,0.6)');
text(X(1),Y(1),Z(1),'A’');
text(-0.25,0.2,-0.3,'C(-0.25,0.2,-0.3)');
title('3D path of Joint Motion')
%畫出路徑上方向
s=1;
for t = -T:sampling:-tacc
    hold on
    plot3([X(s),X(s)+B1(1,3,s)/10],[Y(s),Y(s)+B1(2,3,s)/10],[Z(s),Z(s)+B1(3,3,s)/10]);
    s = s+1;
end
s = 1;
s1 = 151;
for t = (-tacc+sampling):(sampling):(tacc-sampling)
    hold on 
    plot3([X(s1+s),X(s1+s)+B2(1,3,s)/10],[Y(s1+s),Y(s1+s)+B2(2,3,s)/10],[Z(s1+s),Z(s1+s)+B2(3,3,s)/10]);
    s = s+1;
end
s = 1;
s2 = 350;
for t =tacc:sampling:T
    hold on 
    plot3([X(s2+s),X(s2+s)+B3(1,3,s)/10],[Y(s2+s),Y(s2+s)+B3(2,3,s)/10],[Z(s2+s),Z(s2+s)+B3(3,3,s)/10]);
    s = s+1;
end