function [dy]=EquationOfMotions(t,y)

m=2*0.130; %Kg
Iz=0.00190; %Kh m^2
Ix=0.00190;
Iy=0.00000488;
Ixy=0;
Ixz=0;
Iyz=0;
R = 0.3048; %m
chord = 0.0508; %m hp
Irr=0.5*m*R*R;
Ms=0;
g=9.81;

geo.R = R;
geo.c = chord;

% dtheta=y(1);
% dphi=y(2);
% dpsi=y(3);
theta=y(1);
phi=y(2);
psi=y(3);
p=y(4);
q=y(5);
r=y(6);
ux=y(7);
uy=y(8);
uz=y(9);

% %angular_vel_body frame
% p=dphi-dpsi*sin(theta);
% q=dtheta*cos(phi)+dpsi*cos(theta)*sin(phi);
% r=-dtheta*sin(phi)+dpsi*cos(theta)*cos(phi);

T0=[cos(theta)*cos(psi), -cos(theta)*sin(psi), -sin(theta)
    cos(phi)*sin(psi)-sin(phi)*sin(theta)*cos(psi), cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi), -sin(phi)*cos(theta)
    sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi), +sin(phi)*cos(psi)-cos(phi)*sin(theta)*sin(psi), cos(phi)*cos(theta)];

theta0=0;
phi0=70*pi/180;
psi0=45*pi/180;
Tl_0=[cos(theta0)*cos(psi0), -cos(theta0)*sin(psi0), -sin(theta0)
    cos(phi0)*sin(psi0)-sin(phi0)*sin(theta0)*cos(psi0), cos(phi0)*cos(psi0)+sin(phi0)*sin(theta0)*sin(psi0), -sin(phi0)*cos(theta0)
    sin(phi0)*sin(psi0)+cos(phi0)*sin(theta0)*cos(psi0), +sin(phi0)*cos(psi0)-cos(phi0)*sin(theta0)*sin(psi0), cos(phi0)*cos(theta0)];

%gravity force
FG=T0*Tl_0*(-m*g*[0;0;1]);

%from aero_BET
[F,M]=aero([ux;uy;uz],[p;q;r]);


%y=[dtheta dphi dpsi theta phi psi dx dy dz x y z];
dy(1)=q*cos(phi)-r*sin(phi);
dy(2)=p+q*sin(phi)*tan(theta)+r*cos(phi)*tan(theta);
% dy(3)=(M3-I33*dy(2)*cos(y(4))+I33*y(2)*y(1)*sin(y(4)))/I33;
dy(3)=q*sin(phi)/cos(theta)+r*cos(phi)/cos(theta);
M_pqr=[Ix -Ixy -Ixz; -Ixy Iy -Iyz ; -Ixz -Iyz Iz ];
R=[-(Iz-Iy)*q*r-(Ixy*p+Iyz*r)*r+(Ixz*p+Iyz*q)*q+M(1);...
    -(Ix-Iz)*p*r-(Iyz*q+Ixz*p)*p+(Ixy*p+Ixz*r)*r+M(2);...
    -(Iy-Ix)*p*q-(Ixz*r+Ixy*q)*q+(Iyz*r+Ixy*p)*p+M(3)];

dy(4:6)=M_pqr\R;
dy(4)=0;
dy(5)=0;
% dy(4)=[-(Iz-Iy)*q*r-(Ixy*p+Iyz*r)*r+(Ixz*p+Iyz*q)*q+M(1)]/Ix;
% dy(5)=[-(Ix-Iz)*p*r-(Iyz*q+Ixz*p)*p+(Ixy*p+Ixz*r)*r+M(2)]/Iy;
% dy(6)=[-(Iy-Ix)*p*q-(Ixz*r+Ixy*q)*q+(Iyz*r+Ixy*p)*p+M(3)]/Iz;
dy(7)=[-m*q*uz+m*r*uy+F(1)+FG(1)]/m;
dy(8)=[-m*r*ux+m*p*uz+F(2)+FG(2)]/m;
dy(9)=[-m*p*uy+m*q*ux+F(3)+FG(3)]/m;

VEL=Tl_0'*T0'*[ux;uy;uz];

dy(10)=VEL(1);
dy(11)=VEL(2);
dy(12)=VEL(3);
dy=dy';
end