clc
clear

%A B C 's matrix
A = [0  1  0  20;
    -1  0  0  30;
     0  0  1  20;
     0  0  0   1  ];
B = [ 0  0 -1  -10;
     -1  0  0   15;
      0  1  0   30;
      0  0  0    1  ];
C = [ 1  0  0  -25;
      0 -1  0   10;
      0  0 -1  -20;
      0  0  0    1  ];

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
%% the path from A to A'
Var1 = SixVar(A,B);
n = 1;
sampling = 0.002;
tacc = 0.2;
T = 0.5;
%A -A' ?u??
for t = -T:sampling:-tacc
    h = (t+T)/T;
    rx = Var1(1) * h;
    ry = Var1(2) * h;
    rz = Var1(3) * h;
    rpsi = Var1(4) * h;
    rtheta = Var1(5) * h;
    rphi = Var1(6) * h;
    
    Tr = [1 0 0 rx ; 0 1 0 ry ; 0 0 1 rz ; 0 0 0 1];
    Ra = [(s(Var1(4))^2)*Vers(rtheta)+c(rtheta)    ,-s(Var1(4))*c(Var1(4))*Vers(rtheta)                 ,c(Var1(4))*s(rtheta) ,0;
               -s(Var1(4))*c(Var1(4))*Vers(rtheta)                  ,(c(Var1(4))^2)*Vers(rtheta)+c(rtheta)   ,s(Var1(4))*s(rtheta) ,0;
               -c(Var1(4))*s(rtheta)                                    ,-s(Var1(4))*s(rtheta)                                   ,c(rtheta)            ,0;
               0                                                                ,0                                                              ,0                          ,1];
    Ro = [c(rphi) ,-s(rphi) ,0 ,0;
               s(rphi) ,c(rphi)   ,0 ,0;
               0 0 1 0;
               0 0 0 1];
           
    Dr = Tr*Ra*Ro;
    pA(:,:,n) = A*Dr;
    n=n+1;
end
%% the path from A' to C'
A2 = pA(:,:,n-1);
Var2 = SixVar(B,A2);
Var3 = SixVar(B,C);

if abs(Var3(4)-Var2(4)) > pi/2
    Var2(4) = Var2(4) + pi;
    Var2(5) = -Var2(5);
end

n = 1;
for t = -tacc+sampling:sampling:tacc-sampling
    h = (t+tacc)/(2*tacc); 
    rx2 = ((Var3(1)*tacc/T+Var2(1))*(2-h)*h^2-2*Var2(1))*h+Var2(1);
    ry2 = ((Var3(2)*tacc/T+Var2(2))*(2-h)*h^2-2*Var2(2))*h+Var2(2);
    rz2 = ((Var3(3)*tacc/T+Var2(3))*(2-h)*h^2-2*Var2(3))*h+Var2(3);
    rpsi2 = (Var3(4)-Var2(4))*h+Var2(4);
    rtheta2 = ((Var3(5)*tacc/T+Var2(5))*(2-h)*h^2-2*Var2(5))*h+Var2(5);
    rphi2 = ((Var3(6)*tacc/T+Var2(6))*(2-h)*h^2-2*Var2(6))*h+Var2(6);
    
    Tr2 = [1 0 0 rx2 ; 0 1 0 ry2 ; 0 0 1 rz2 ; 0 0 0 1];
    Ra2 = [(s(rpsi2)^2)*Vers(rtheta2)+c(rtheta2)    ,-s(rpsi2)*c(rpsi2)*Vers(rtheta2)                 ,c(rpsi2)*s(rtheta2) ,0;
               -s(rpsi2)*c(rpsi2)*Vers(rtheta2)                  ,(c(rpsi2)^2)*Vers(rtheta2)+c(rtheta2)   ,s(rpsi2)*s(rtheta2) ,0;
               -c(rpsi2)*s(rtheta2)                                    ,-s(rpsi2)*s(rtheta2)                                   ,c(rtheta2)            ,0;
               0                                                                ,0                                                              ,0                          ,1];
    Ro2 = [c(rphi2) ,-s(rphi2) ,0 ,0;
               s(rphi2) ,c(rphi2)   ,0 ,0;
               0 0 1 0;
               0 0 0 1];
           
    Dr2 = Tr2*Ra2*Ro2;
    pAC(:,:,n) = B*Dr2;
    n=n+1;
end

%% the path from C' to C
n = 1;
for t = tacc:sampling:T
    h = t/T;
    rx3 = Var3(1) * h;
    ry3 = Var3(2) * h;
    rz3 = Var3(3) * h;
    rpsi3 = Var3(4) ;
    rtheta3 = Var3(5) * h;
    rphi3 = Var3(6) * h;
    
    Tr3 = [1 0 0 rx3 ; 0 1 0 ry3 ; 0 0 1 rz3 ; 0 0 0 1];
    Ra3 = [(s(rpsi3)^2)*Vers(rtheta3)+c(rtheta3)    ,-s(rpsi3)*c(rpsi3)*Vers(rtheta3)                 ,c(rpsi3)*s(rtheta3) ,0;
               -s(rpsi3)*c(rpsi3)*Vers(rtheta3)                  ,(c(rpsi3)^2)*Vers(rtheta3)+c(rtheta3)   ,s(rpsi3)*s(rtheta3) ,0;
               -c(rpsi3)*s(rtheta3)                                    ,-s(rpsi3)*s(rtheta3)                                   ,c(rtheta3)            ,0;
               0                                                                ,0                                                              ,0                          ,1];
    Ro3 = [c(rphi3) ,-s(rphi3) ,0 ,0;
               s(rphi3) ,c(rphi3)   ,0 ,0;
               0 0 1 0;
               0 0 0 1];
           
    Dr3 = Tr3*Ra3*Ro3;
    pC(:,:,n) = B*Dr3;
    n=n+1;
end
%%   ?e?Xposition velocity acceleration ????
%???T?q??x y z ?X?_????X????
xA(:) = pA(1,4,:);
xB(:) = pAC(1,4,:);
xC(:) = pC(1,4,:);

yA(:) = pA(2,4,:);
yB(:) = pAC(2,4,:);
yC(:) = pC(2,4,:);

zA(:) = pA(3,4,:);
zB(:) = pAC(3,4,:);
zC(:) = pC(3,4,:);

X = [xA xB xC];
Y = [yA yB yC];
Z = [zA zB zC];

t = -T:sampling:T;
figure(1)
subplot(3,3,1);
plot(t,X);
grid on
title('position of X');
ylabel('m')
subplot(3,3,4);
plot(t,Y);
grid on
title('position of Y');
ylabel('m')
subplot(3,3,7);
plot(t,Z);
grid on
title('position of Z')
ylabel('m')

dt = t(2:501);
dX = diff(X)/sampling;
dY = diff(Y)/sampling;
dZ = diff(Z)/sampling;

subplot(3,3,2);
plot(dt,dX);
grid on
title('velocity of X')
ylabel('m/s')
subplot(3,3,5);
plot(dt,dY);
grid on
title('velocity of Y')
ylabel('m/s')
subplot(3,3,8);
plot(dt,dZ);
grid on
title('velocity of Z')
ylabel('m/s')

ddt = t(3:501);
ddX = diff(dX)/sampling;
ddY = diff(dY)/sampling;
ddZ = diff(dZ)/sampling;

subplot(3,3,3);
plot(ddt,ddX);
grid on
title('acceleration of X')
ylabel('m/s^2')
subplot(3,3,6);
plot(ddt,ddY);
grid on
title('acceleration of Y')
ylabel('m/s^2')
subplot(3,3,9);
plot(ddt,ddZ);
grid on
title('acceleration of Y')
ylabel('m/s^2')

%%?e?X???|
figure(2)

subplot(1,2,1);
plot3(X,Y,Z);
grid on

hold on;
plot3([pax,pax+nax/10],[pay,pay+nay/10],[paz,paz+naz/10]);
hold on;
plot3([pax,pax+oax/10],[pay,pay+oay/10],[paz,paz+oaz/10]);
hold on;
plot3([pax,pax+aax/10],[pay,pay+aay/10],[paz,paz+aaz/10]);
hold on;
plot3([pbx,pbx+nbx/10],[pby,pby+nby/10],[pbz,pbz+nbz/10]);
hold on;
plot3([pbx,pbx+obx/10],[pby,pby+oby/10],[pbz,pbz+obz/10]);
hold on;
plot3([pbx,pbx+abx/10],[pby,pby+aby/10],[pbz,pbz+abz/10]);
hold on;
plot3([pcx,pcx+ncx/10],[pcy,pcy+ncy/10],[pcz,pcz+ncz/10]);
hold on;
plot3([pcx,pcx+ocx/10],[pcy,pcy+ocy/10],[pcz,pcz+ocz/10]);
hold on;
plot3([pcx,pcx+acx/10],[pcy,pcy+acy/10],[pcz,pcz+acz/10]);
hold on;

%plot3([X(151),pB(1)],[yA_B(151),pB(2)],[zA_B(151),pB(3)],':');
%hold on;
%plot3([x_C(1),pB(1)],[y_C(1),pB(2)],[z_C(1),pB(3)],':');
xlabel('x(m)');
ylabel('y(m)');
zlabel('z(m)');
text(-0.3,0.2,0.3,'A(-0.3,0.2,0.3)');
text(X(151),Y(151),Z(151),'A??');
text(0.5,-0.2,0.6,'B(0.5,-0.2,0.6)');
text(-0.25,0.2,-0.3,'C(-0.25,0.2,-0.3)');
title('3D path of Cartesion Motion')

subplot(1,2,2);
plot3(X,Y,Z);
grid on

hold on;
plot3([pax,pax+nax/10],[pay,pay+nay/10],[paz,paz+naz/10]);
hold on;
plot3([pax,pax+oax/10],[pay,pay+oay/10],[paz,paz+oaz/10]);
hold on;
plot3([pax,pax+aax/10],[pay,pay+aay/10],[paz,paz+aaz/10]);
hold on;
plot3([pbx,pbx+nbx/10],[pby,pby+nby/10],[pbz,pbz+nbz/10]);
hold on;
plot3([pbx,pbx+obx/10],[pby,pby+oby/10],[pbz,pbz+obz/10]);
hold on;
plot3([pbx,pbx+abx/10],[pby,pby+aby/10],[pbz,pbz+abz/10]);
hold on;
plot3([pcx,pcx+ncx/10],[pcy,pcy+ncy/10],[pcz,pcz+ncz/10]);
hold on;
plot3([pcx,pcx+ocx/10],[pcy,pcy+ocy/10],[pcz,pcz+ocz/10]);
hold on;
plot3([pcx,pcx+acx/10],[pcy,pcy+acy/10],[pcz,pcz+acz/10]);
hold on;

xlabel('x(m)');
ylabel('y(m)');
zlabel('z(m)');
text(-0.3,0.2,0.3,'A(-0.3,0.2,0.3)');
text(X(151),Y(151),Z(151),'A??');
text(0.5,-0.2,0.6,'B(0.5,-0.2,0.6)');
text(-0.25,0.2,-0.3,'C(-0.25,0.2,-0.3)');
title('3D path of Cartesion Motion')

n = 1;
for t = -T:sampling:-tacc
    hold on;
    plot3([X(n),X(n)+pA(1,3,n)/20],[Y(n),Y(n)+pA(2,3,n)/20],[Z(n),Z(n)+pA(3,3,n)/20]);
    n = n+1;
end
n = 1;
n1 = 151;
for t = (-tacc+sampling):sampling:(tacc-sampling)
    hold on;
    plot3([X(n1+n),X(n1+n)+pAC(1,3,n)/20],[Y(n1+n),Y(n1+n)+pAC(2,3,n)/20],[Z(n1+n),Z(n1+n)+pAC(3,3,n)/20]);
    n = n+1;
end
n = 1;
n2 = 350;
for t = tacc:sampling:T
    hold on;
    plot3([X(n2+n),X(n2+n)+pC(1,3,n)/20],[Y(n2+n),Y(n2+n)+pC(2,3,n)/20],[Z(n2+n),Z(n2+n)+pC(3,3,n)/20]);
    n = n+1;
end
%% helping function
function A = s(theta)
A = sin(theta);
end

function A = c(theta)
A = cos(theta);
end

function A = Vers(theta)
A = 1-c(theta);
end

function A = SixVar(A,B)
n1 = [A(1,1);A(2,1);A(3,1)];
n2 = [B(1,1);B(2,1);B(3,1)];
o1 = [A(1,2);A(2,2);A(3,2)];
o2 = [B(1,2);B(2,2);B(3,2)];
a1 = [A(1,3);A(2,3);A(3,3)];
a2 = [B(1,3);B(2,3);B(3,3)];
p1 = [A(1,4);A(2,4);A(3,4)];
p2 = [B(1,4);B(2,4);B(3,4)];

x = dot(n1,(p2-p1));
y = dot(o1,(p2-p1));
z = dot(a1,(p2-p1));
psi = atan2(dot(o1,a2),dot(n1,a2));
theta = atan2(sqrt((dot(n1,a2))^2+(dot(o1,a2))^2),dot(a1,a2));
Sphi = -s(psi)*c(psi)*Vers(theta)*dot(n1,n2)+(c(psi)^2*Vers(theta)+c(theta))*dot(o1,n2)-s(psi)*s(theta)*dot(a1,n2);
Cphi = -s(psi)*c(psi)*Vers(theta)*dot(n1,o2)+(c(psi)^2*Vers(theta)+c(theta))*dot(o1,o2)-s(psi)*s(theta)*dot(a1,o2);
phi= atan2(Sphi,Cphi);

A = [x y z psi theta phi];
end