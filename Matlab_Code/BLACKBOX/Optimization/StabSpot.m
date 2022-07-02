function Dist = StabSpot(BoomInfo,x,Chi,D,theta)
r0=x(1)*2*pi/10;
phi=x(2)*pi/180/10;
R=norm(BoomInfo.Aero.P_Finish_Dx);
Vs=r0*R*(1/Chi-1);
z0=1.8;
tfin=10;
[quat,ustart] = HandInitial(r0,theta,D,phi,Vs,eye(3),BoomInfo);
options = odeset('Events', @EventsQUAT,'RelTol',1e-3,'AbsTol',1e-5);
Y0=[quat 0 0 r0  ustart(1) ustart(2) ustart(3) 0 0 z0 ]';
[~,YOUT_quat] = ode45(@(t,y)EquationOfMotionsQuaternion(t,y,BoomInfo,eye(3)),[0 tfin],Y0,options); %
Dist=norm(YOUT_quat(end,11:13));
if max(vecnorm(YOUT_quat(:,11:13)'))/1.1<=Dist
    Dist=1000;
end
end

