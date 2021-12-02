function Dr = calDr(x,y,z,psi,theta,phi,r)

Dr = zeros(4,4);
Dr(:,4) = [r*x;r*y;r*z;1];
Dr(:,3) = [cos(psi)*sin(r*theta);sin(psi)*sin(r*theta);cos(r*theta);0];
Dr(:,2) = [-sin(r*theta)*(sin(psi)^2*(1-cos(r*theta))+cos(r*theta)) + cos(r*phi)*(-sin(psi)*cos(psi)*(1-cos(r*theta)));
           -sin(r*theta)*(-sin(psi)*cos(psi)*(1-cos(r*theta))) + cos(r*phi)*(cos(psi)^2*(1-cos(r*theta))+cos(r*theta));
           -sin(r*theta)*(-cos(psi)*sin(r*theta)) + cos(r*theta)*(-sin(psi)*sin(r*theta));
           0];
Dr(1:3,1) = cross(Dr(1:3,2), Dr(1:3,3));