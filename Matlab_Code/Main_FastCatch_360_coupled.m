%% Script per creare la Geometria 3D
%Main accoppiato con XFOIL (NON FUNZIONANTE) senza GA
clear all 
close all
addpath(genpath('BLACKBOX'));
%% Input Data goemtrici 3D
Chord=0.05;
p_c=20; % numero di profili di "Transizione" nella parte centrale
l=0.3; % lunghezza della pala avente un profilo 2D definito, NON corrisponde alla lunghezza del boomerang
delta=40*pi/180; %Angolo di freccia
beta=1*pi/180; %Angolo di Diedro
pitch=3*pi/180; %Pitch angle
num=20; %Numero di profili totale su ciascuna metà;
PARA=2.; %Parametro che permette di modificare la curvatura centrale (più si avvicna ad 1 pù dietro forma una V
% Profile 2D Shape
%% Profilo 2D e caratteristiche aerodinamiche
Profile2D.data=importdata('Eppler_61.dat');
Profile2D.data=[Profile2D.data(1:33,:); Profile2D.data(34:60,:)];
Xp = Profile2D.data(:,1).*Chord-Chord;
Zp = Profile2D.data(:,2).*Chord-max(Profile2D.data(:,2).*Chord)/2;
Xp_flip = -(Chord/2.*ones(size(Xp))+Xp)+Chord/2.*ones(size(Xp))-Chord;
Xp_flip = [Xp_flip(numel(Xp_flip)/2+1:end); Xp_flip(1:numel(Xp_flip)/2)];
Zp_flip = [Zp(numel(Xp_flip)/2+1:end); Zp(1:numel(Xp_flip)/2)];
%% Creazione di BoomInfo.s
BoomInfo.Pianta.l=l;
BoomInfo.Pianta.freccia=delta;
BoomInfo.Pianta.diedro=beta;
BoomInfo.Pianta.pitch=pitch;
BoomInfo.Geom3D.p_c=p_c;
BoomInfo.Geom3D.num=num;
BoomInfo.Geom3D.PARA=PARA;
BoomInfo.Mecc.Dens=650;
BoomInfo.Profile.Chord=Chord;
BoomInfo.Profile.Xp_sx=Xp;
BoomInfo.Profile.Xp_dx=Xp_flip;
BoomInfo.Profile.Zp_sx=Zp;
BoomInfo.Profile.Zp_dx=Zp_flip;
CheckBoomInfo(BoomInfo)
%% Creazione della geometria tridemensionale
[BoomInfo] = Boom3DShape(BoomInfo,'Info','Create_Stl');
%% Calcolo coefficienti aerodinamici per il profilo scelto
coeff360  = f_polar_360('load', 'fastcatch.dat',linspace(-10,16, 27), 50000,2,5, 0.007, 1.6);
CL_t      = coeff360.CL;
CD_t      = coeff360.CD;
CM_t      = coeff360.CM;
alpha_cl  = coeff360.alpha;
alpha_cd  = coeff360.alpha;
alpha_cm  = coeff360.alpha;
BoomInfo.Aero.alpha_cl=alpha_cl;
BoomInfo.Aero.alpha_cd=alpha_cd;
BoomInfo.Aero.alpha_cm=alpha_cm;
BoomInfo.Aero.Cl=CL_t;
BoomInfo.Aero.Cd=CD_t;
BoomInfo.Aero.Cm=CM_t;
%%
CheckBoomInfo(BoomInfo,'Plot')
%% Initial Condition
X_ini=[6.0    0    1.6000   65.5000    8.3000]*10;
r0=X_ini(1)*2*pi/10;
theta=X_ini(2)*pi/180/10;
D=X_ini(3)*pi/180/10;
phi=X_ini(4)*pi/180/10;
Vs=X_ini(5)/10;
z0= 1.8; % initial altitude
[quat,ustart] = HandInitial(r0,theta,D,phi,Vs,BoomInfo);
tfin=40;
%[V_dx_b,V_sx_b]=InitialConditionPlot(quatToAtt(quat),ustart',[0;0;r0],BoomInfo);
%%
options = odeset('Events', @EventsQUAT,'RelTol',1e-4,'AbsTol',1e-6);
Y0=[quat 0 0 r0  ustart(1) ustart(2) ustart(3) 0 0 z0 ]';
tic
[TOUT,YOUT_quat] = ode45(@(t,y)EquationOfMotionsQuaternion(t,y,BoomInfo),[0 tfin],Y0,options); %
toc
%% Grafici Finali
[YOUT] = Eul_Quat(YOUT_quat,TOUT);
Energy(TOUT,YOUT,BoomInfo)
PlotTipDxSx(TOUT,YOUT,BoomInfo)
PlotAeroForce(YOUT,TOUT,BoomInfo)
ChiAvan(BoomInfo,YOUT,TOUT)
FinalReport(YOUT,TOUT)
