function [PAR,varargout] = GA_para_paper(x,BoomInfo,varargin)

BoomInfo.Mecc.Dens=100;
BoomInfo.Geom3D.PARA=x/1000; %Parametro che permette di modificare la curvatura centrale (più si avvicna ad 1 pù dietro forma una V

[BoomInfo] = Boom3DShape(BoomInfo);
D=0.130/BoomInfo.Mecc.V;
%BoomInfo.Mecc.CG=[CG(1) 0 0];
BoomInfo.Mecc.Dens=D;

[BoomInfo] = Boom3DShape(BoomInfo);

theta0=0*pi/180;
phi0=0*pi/180;
psi0=0*pi/180;
Tl_0=[cos(theta0)*cos(psi0), cos(theta0)*sin(psi0), -sin(theta0)
    -cos(phi0)*sin(psi0)+sin(phi0)*sin(theta0)*cos(psi0), cos(phi0)*cos(psi0)+sin(phi0)*sin(theta0)*sin(psi0), sin(phi0)*cos(theta0)
    sin(phi0)*sin(psi0)+cos(phi0)*sin(theta0)*cos(psi0), -sin(phi0)*cos(psi0)+cos(phi0)*sin(theta0)*sin(psi0), cos(phi0)*cos(theta0)];
z0= 1.8; % initial altitude
tfin=40;
r0=10*2*pi;
psi=70*pi/180;
theta=0*pi/180;
phi=-45*pi/180;
eul=[psi theta phi];
quat = eul2quat( eul );
quat=[quat(2) quat(3) quat(4) quat(1)];
ustart=[25;0;0];
options = odeset('Events', @EventsQUAT,'RelTol',1e-4,'AbsTol',1e-6);
Y0=[quat 0 0 r0  ustart(1) ustart(2) ustart(3) 0 0 z0 ]';
%%
[TOUT,YOUT_quat] = ode45(@(t,y)EquationOfMotionsQuaternion(t,y,BoomInfo,Tl_0),[0 tfin],Y0,options); %

% La distanza ed il tempo vengono minimizzati
Dist=norm(YOUT_quat(end,11:13));

if max(vecnorm(YOUT_quat(:,11:13)'))/1.1<=Dist
    Dist=1000;
end
PAR=Dist;
if isnan(Dist)
    fprintf('Lol \n');
end
%PAR=Dist;
if not(isempty(varargin))
    PAR(1)=Dist;
    PAR(2)=TOUT(end);
    fprintf ('SIMULAZIONE senza velocità indotta:\n');
    fprintf('Rotational Rate %.2f, Speed Tip %.2f \n',r0/2/pi,Vs);
    [YOUT] = Eul_Quat(YOUT_quat,TOUT);
    [t_min,r_max]=FinalReport(YOUT,TOUT);
    varargout{1}=t_min;
    varargout{2}=r_max;
end

