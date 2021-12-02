function [x, y, z, psi, theta, phi] = calD1(pos1, pos2)

x = pos1(:,1)' * (pos2(:,4) - pos1(:,4));
y = pos1(:,2)' * (pos2(:,4) - pos1(:,4));
z = pos1(:,3)' * (pos2(:,4) - pos1(:,4));
psi = atan2(pos1(:,2)'*pos2(:,3), pos1(:,1)'*pos2(:,3));
theta = atan2(sqrt((pos1(:,1)'*pos2(:,3))^2 + (pos1(:,2)'*pos2(:,3))^2), pos1(:,3)'*pos2(:,3));

phiS = -sin(psi)*cos(psi)*(1-cos(theta))*pos1(:,1)'*pos2(:,1) ...
       +(cos(psi)^2*(1-cos(theta))+cos(theta))*pos1(:,2)'*pos2(:,1) ...
       - sin(psi)*sin(theta)*pos1(:,3)'*pos2(:,1);
phiC = -sin(psi)*cos(psi)*(1-cos(theta))*pos1(:,1)'*pos2(:,2) ...
       +(cos(psi)^2*(1-cos(theta))+cos(theta))*pos1(:,2)'*pos2(:,2) ...
       - sin(psi)*sin(theta)*pos1(:,3)'*pos2(:,2);
phi = atan2(phiS, phiC);