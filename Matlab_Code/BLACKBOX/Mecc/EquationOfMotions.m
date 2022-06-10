function [dy]=EquationOfMotions(t,y,fileID, BoomInfo)
%This function calculates,for every point of the trajectory,the
% angular velocities(dtheta,dphi,dpsi),the angular accellerations in the body
% frame(dp,dr,dq)and the 3 components of velocity(ux,uy,uz)

%t
I=BoomInfo.Mecc.I_rho;
m=BoomInfo.Mecc.m;

g=9.81;


Iz=I(3,3);
Ix= I(1,1);
Iy=I(2,2);
Ixy=I(1,2);
Ixz=I(1,3);
Iyz=I(2,3);
% Iz=0.00588; %Kh m^2
% Ix=0.00490;%0.00407;
% Iy=0.000995;%0.0096;
% Ixy=0.0002;
% Ixz=0;
% Iyz=0;

theta=y(1);
phi=y(2);
psi=y(3);
p=y(4);
q=y(5);
r=y(6);
ux=y(7);
uy=y(8);
uz=y(9);



T0=[cos(theta)*cos(psi), cos(theta)*sin(psi), -sin(theta)
    -cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi), cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi), sin(phi)*cos(theta)
    sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi), -sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi), cos(phi)*cos(theta)];
FG=T0*(-m*g*[0;0;1]);


[F,M]=AeroDynamics([ux;uy;uz],[p;q;r],BoomInfo);


dy(1)=q*cos(phi)-r*sin(phi);
dy(2)=p+q*sin(phi)*tan(theta)+r*cos(phi)*tan(theta);
dy(3)=q*sin(phi)/cos(theta)+r*cos(phi)/cos(theta);
M_pqr=[Ix -Ixy -Ixz; -Ixy Iy -Iyz ; -Ixz -Iyz Iz ];
R=[-(Iz-Iy)*q*r-(Ixy*p+Iyz*r)*r+(Ixz*p+Iyz*q)*q+M(1);...
    -(Ix-Iz)*p*r-(Iyz*q+Ixz*p)*p+(Ixy*p+Ixz*r)*r+M(2);...
    -(Iy-Ix)*p*q-(Ixz*r+Ixy*q)*q+(Iyz*r+Ixy*p)*p+M(3)];

dy(4:6)=M_pqr\R;
dy(7)=(-m*q*uz+m*r*uy+F(1)+FG(1))/m;
dy(8)=(-m*r*ux+m*p*uz+F(2)+FG(2))/m;
dy(9)=(-m*p*uy+m*q*ux+F(3)+FG(3))/m;

VEL=T0'*[ux;uy;uz];

dy(10)=VEL(1);
dy(11)=VEL(2);
dy(12)=VEL(3);
dy=dy';

% fprintf(fileID,'TIME: %.5f \n',t);
% fprintf(fileID,'dTheta: %.f ',dy(1));
% fprintf(fileID,'dPhi: %.5f ',dy(2));
% fprintf(fileID,'dPsi: %.5f ',dy(3));
% fprintf(fileID,'dp: %.5f ',dy(4));
% fprintf(fileID,'dq: %.5f ',dy(5));
% fprintf(fileID,'dr: %.5f ',dy(6));
% fprintf(fileID,'ux: %.5f ',VEL(1));
% fprintf(fileID,'uy: %.5f ',VEL(2));
% fprintf(fileID,'uz: %.5f ',VEL(3));



end