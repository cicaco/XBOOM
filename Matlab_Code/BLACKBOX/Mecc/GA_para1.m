function [PAR] = GA_para1(x,BoomInfo,varargin)

r0=x(1)*2*pi/10;
theta=x(2)*pi/180/10;
D=x(3)*pi/180/10;
phi=x(4)*pi/180/10;
Vs=x(5)/10;


theta0=0*pi/180;
phi0=0*pi/180;
psi0=0*pi/180;
Tl_0=[cos(theta0)*cos(psi0), cos(theta0)*sin(psi0), -sin(theta0)
    -cos(phi0)*sin(psi0)+sin(phi0)*sin(theta0)*cos(psi0), cos(phi0)*cos(psi0)+sin(phi0)*sin(theta0)*sin(psi0), sin(phi0)*cos(theta0)
    sin(phi0)*sin(psi0)+cos(phi0)*sin(theta0)*cos(psi0), -sin(phi0)*cos(psi0)+cos(phi0)*sin(theta0)*sin(psi0), cos(phi0)*cos(theta0)];
z0= 1.8; % initial altitude
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

tfin=40;


%[V_dx_b,V_sx_b]=InitialConditionPlot(Tl_0,T0,ustart,[0;0;r0],BoomInfo);


options = odeset('Events', @EventsQUAT,'RelTol',1e-2,'AbsTol',1e-3);
Y0=[quat 0 0 r0  ustart(1) ustart(2) ustart(3) 0 0 z0 ]';
%%
%[TOUT,YOUT_quat] = ode45(@(t,y)EquationOfMotionsQuaternion(t,y,BoomInfo,Tl_0),[0 tfin],Y0,options); %
[TOUT,YOUT_quat] = ode45(@(t,y)TestEquationOfMotionsQuaternion(t,y,BoomInfo,Tl_0),[0 tfin],Y0,options); %

Dist=norm(YOUT_quat(end,11:13));

if max(vecnorm(YOUT_quat(:,11:13)'))/1.1<=Dist
    Dist=1000;
end
PAR=Dist;
%PAR=Dist;
if not(isempty(varargin))
    PAR(1)=Dist;
    PAR(2)=TOUT(end);
    fprintf('Rotational Rate %.2f, Speed Tip %.2f',r0/2/pi,Vs);
    
end

