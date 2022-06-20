function [dy]=EquationOfMotionsQuaternion(t,y,fileID, BoomInfo,Tl_0)
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

q1=y(1);
q2=y(2);
q3=y(3);
q4=y(4);
p=y(5);
q=y(6);
r=y(7);
ux=y(8);
uy=y(9);
uz=y(10);
quat=[q1 q2 q3 q4];
T0 = quatToAtt( quat );
% 
% T0=[cos(theta)*cos(psi), cos(theta)*sin(psi), -sin(theta)
%     -cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi), cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi), sin(phi)*cos(theta)
%     sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi), -sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi), cos(phi)*cos(theta)];

FG=T0*Tl_0*(-m*g*[0;0;1]);


[F,M]=AeroDynamics([ux;uy;uz],[p;q;r],BoomInfo);

dy(1:4)=1/2*[0 r -q p; -r 0 p q; q -p 0 r; -p -q -r 0]*[q1 q2 q3 q4]';
% dy(1)=q*cos(phi)-r*sin(phi);
% dy(2)=p+q*sin(phi)*tan(theta)+r*cos(phi)*tan(theta);
% dy(3)=q*sin(phi)/cos(theta)+r*cos(phi)/cos(theta);
M_pqr=[Ix -Ixy -Ixz; -Ixy Iy -Iyz ; -Ixz -Iyz Iz ];
R=[-(Iz-Iy)*q*r-(Ixy*p+Iyz*r)*r+(Ixz*p+Iyz*q)*q+M(1);...
    -(Ix-Iz)*p*r-(Iyz*q+Ixz*p)*p+(Ixy*p+Ixz*r)*r+M(2);...
    -(Iy-Ix)*p*q-(Ixz*r+Ixy*q)*q+(Iyz*r+Ixy*p)*p+M(3)];

dy(5:7)=M_pqr\R;
dy(8)=(-m*q*uz+m*r*uy+F(1)+FG(1))/m;
dy(9)=(-m*r*ux+m*p*uz+F(2)+FG(2))/m;
dy(10)=(-m*p*uy+m*q*ux+F(3)+FG(3))/m;

VEL=Tl_0'*T0'*[ux;uy;uz];

dy(11)=VEL(1);
dy(12)=VEL(2);
dy(13)=VEL(3);
dy=dy';

% fprintf(fileID,'TIME: %.5f \n',t);
% fprintf(fileID,'Re: %.5f ',Re); 

% fprintf(fileID,' %.5f \n ',AoA1(1:end)*180/pi); 



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