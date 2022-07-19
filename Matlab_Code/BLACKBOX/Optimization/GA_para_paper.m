function [PAR,varargout] = GA_para_paper(x,BoomInfo,varargin)

BoomInfo.Mecc.Dens=300;
BoomInfo.Geom3D.PARA=x(1)/1000; %Parametro che permette di modificare la curvatura centrale (più si avvicna ad 1 pù dietro forma una V
BoomInfo.Geom3D.l=x(2)/100; %Parametro che permette di modificare la curvatura centrale (più si avvicna ad 1 pù dietro forma una V
BoomInfo.Pianta.freccia=x(3)/100*pi/180; 
[BoomInfo] = Boom3DShape(BoomInfo);
D=0.0620/BoomInfo.Mecc.V;
%BoomInfo.Mecc.CG=[CG(1) 0 0];
BoomInfo.Mecc.Dens=D;

[BoomInfo] = Boom3DShape(BoomInfo);
X_ini=[9.7    5    0   45.000    8.3000];
r0=X_ini(1)*2*pi;
theta=X_ini(2)*pi/180;
D=X_ini(3)*pi/180;
phi=X_ini(4)*pi/180;
Vs=X_ini(5);
z0= 1.8; % initial altitude
[quat,~] = HandInitial(r0,theta,D,phi,Vs,BoomInfo);
TO=quatToAtt(quat);
ustart=TO*[16.5*cos(5*pi/180);0;16.5*sin(5*pi/180)];
tfin=8;
Y0=[quat 0 0 r0  ustart(1) ustart(2) ustart(3) 0 0 z0 ]';
options = odeset('Events', @EventsQUAT,'RelTol',1e-4,'AbsTol',1e-6);

[TOUT,YOUT_quat] = ode45(@(t,y)EquationOfMotionsQuaternion(t,y,BoomInfo),[0 tfin],Y0,options); %

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

