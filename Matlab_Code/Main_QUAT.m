%% Script per creare la Geometria 3D
clear all 
close all
addpath(genpath('BLACKBOX'));
%% Input Data
Chord=0.0488;
p_c=20; % numero di profili di "Transizione" nella parte centrale
l=0.3; % lunghezza della pala avente un profilo 2D definito, NON corrisponde alla lunghezza del boomerang
delta=30*pi/180; %Angolo di freccia
beta=0.5*pi/180; %Angolo di Diedro
pitch=4*pi/180; %Pitch angle
num=20; %Numero di profili totale su ciascuna metà;
PARA=2.; %Parametro che permette di modificare la curvatura centrale (più si avvicna ad 1 pù dietro forma una V
% Profile 2D Shape
%% Profilo 2D flip e analisi
Profile2D=importdata('Naca0006.dat');
Xp=-[0; fliplr(Profile2D(1:65,1)')' ; fliplr(Profile2D(66:end,1)')'].*Chord;
Zp=[0 ;fliplr(Profile2D(1:65,2)')' ; fliplr(Profile2D(66:end,2)')'].*Chord;
Xp_flip=-(Chord/2.*ones(size(Xp))+Xp)+Chord/2.*ones(size(Xp))-Chord;
Zp_flip=(Zp);
[n,~]=size(Xp);
%Clock-wise direction regeneration
Xp_flip=[fliplr(Xp_flip(1:n/2)')';fliplr(Xp_flip(n/2+1:end)')'];
Zp_flip=[fliplr(Zp_flip(1:n/2)')';fliplr(Zp_flip(n/2+1:end)')'];
figure()
plot(Xp,Zp,'r');
hold on
plot(Xp_flip,Zp_flip,'b');
axis equal
set(gca,'Xdir','reverse')
%% Creazione dell Info Box
alpha_cm=[-180 -150 -90 0 90 150 180];
CM_t=[0 0.4 0.4 0 -0.4 -0.4 0];
T=readmatrix('NACA_0006_T1_Re0.050_M0.00_N9.0_360_M.dat');
alpha_cd=T(13:383,1);
alpha_cl=T(13:383,1);
CD_t=T(13:383,3);
CL_t=T(13:383,2);
figure()
plot(alpha_cl,CL_t,'--r');
hold on
plot(alpha_cd,CD_t,'.-b');
plot(alpha_cm,CM_t,'k');
grid on
legend('CL','CD','CM');
BoomInfo.Aero.alpha_cl=alpha_cl;
BoomInfo.Aero.alpha_cd=alpha_cd;
BoomInfo.Aero.alpha_cm=alpha_cm;
BoomInfo.Aero.Cl=CL_t;
BoomInfo.Aero.Cd=CD_t;
BoomInfo.Aero.Cm=CM_t;
%% Creazione dell Info Box
BoomInfo.Pianta.l=l;
BoomInfo.Pianta.freccia=delta;
BoomInfo.Pianta.diedro=beta;
BoomInfo.Pianta.pitch=pitch;
BoomInfo.Geom3D.p_c=p_c;
BoomInfo.Geom3D.num=num;
BoomInfo.Geom3D.PARA=PARA;
BoomInfo.Profile.Chord=Chord;
BoomInfo.Profile.Xp_dx=Xp;
BoomInfo.Profile.Xp_sx=Xp_flip;
BoomInfo.Profile.Zp_dx=Zp;
BoomInfo.Profile.Zp_sx=Zp_flip;

%%
n=num+p_c;
R=1000;
Dens_i=[R.*ones(1,n-p_c-1) R*ones(1,2*p_c-1) R.*ones(1,n-p_c-1)];
[BoomInfo] = Boom3DShape(BoomInfo,'Info','Density_variation',Dens_i);
BoomInfo.Mecc.I_rho
BoomInfo.Mecc.m
%% Initial condition
theta0=0.0*pi/180;
phi0=0*pi/180;
psi0=0*pi/180;
Tl_0=[cos(theta0)*cos(psi0), cos(theta0)*sin(psi0), -sin(theta0)
    -cos(phi0)*sin(psi0)+sin(phi0)*sin(theta0)*cos(psi0), cos(phi0)*cos(psi0)+sin(phi0)*sin(theta0)*sin(psi0), sin(phi0)*cos(theta0)
    sin(phi0)*sin(psi0)+cos(phi0)*sin(theta0)*cos(psi0), -sin(phi0)*cos(psi0)+cos(phi0)*sin(theta0)*sin(psi0), cos(phi0)*cos(theta0)];
r0=10*2*pi;
theta=0;
psi=0;
phi=0;
T0=[cos(theta)*cos(psi), cos(theta)*sin(psi), -sin(theta)
    -cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi), cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi), sin(phi)*cos(theta)
    sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi), -sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi), cos(phi)*cos(theta)];

z0= 1.8; % initial altitude
tfin=40;
%[quat,ustart] = HandInitial(r0,theta,D,phi,Vs,Tl_0);
eul=[0 0 0];
quat = eul2quat( eul );
quat=[quat(2) quat(3) quat(4) quat(1)];
ustart=[10 0 0];
[V_dx_b,V_sx_b]=InitialConditionPlot(Tl_0,T0,ustart',[0;0;r0],BoomInfo);
options = odeset('Events', @EventsQUAT,'RelTol',1e-4,'AbsTol',1e-6);
Y0=[quat 0 0 r0  ustart(1) ustart(2) ustart(3) 0 0 z0 ]';
%%
tic
[TOUT,YOUT_quat] = ode45(@(t,y)EquationOfMotionsQuaternion(t,y,BoomInfo,Tl_0),[0 tfin],Y0,options); %
toc
[YOUT] = Eul_Quat(YOUT_quat,TOUT);
Energy(TOUT,YOUT,BoomInfo)
PlotTipDxSx(TOUT,YOUT,BoomInfo,Tl_0)
PlotAeroForce(YOUT,TOUT,BoomInfo)
ChiAvan(BoomInfo,YOUT,TOUT)
FinalReport(YOUT,TOUT)
