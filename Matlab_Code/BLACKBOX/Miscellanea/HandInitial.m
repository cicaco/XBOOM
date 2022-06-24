function [quat,ustart] = HandInitial(r0,theta,D,phi,Vs,Tl_0,BoomInfo)
%r0=10*2*pi;
% theta=10*pi/180;
% D=60*pi/180;
% phi=85*pi/180;
% Vs=15;
psi=pi-D;
T0=[cos(theta)*cos(psi), cos(theta)*sin(psi), -sin(theta)
    -cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi), cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi), sin(phi)*cos(theta)
    sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi), -sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi), cos(phi)*cos(theta)];
V_tip=(T0*Tl_0*[Vs*cos(theta)*cos(D);-Vs*cos(theta)*sin(D);Vs*sin(theta)])'; %Velocità della tip nel piano del boomerang
r_mano=[0 0 r0];
P_tip=BoomInfo.Aero.P_Finish_Dx;
ustart=V_tip+cross(r_mano,-P_tip');
eul=[psi theta phi];
quat = eul2quat( eul );
quat=[quat(2) quat(3) quat(4) quat(1)];

end

