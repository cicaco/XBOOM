%% Script per creare la Geometria 3D
% Questo main presenta un caso standard, con il calcolo di stabilità a
% "Macchia" tramite la funzione StabilityCheck
clear all 
close all
addpath(genpath('BLACKBOX'));
%% Input Data
%% Input Data
p_c=10; % numero di profili di "Transizione" nella parte centrale
l=14/50; % lunghezza della pala avente un profilo 2D definito, NON corrisponde alla lunghezza del boomerang
Chord=l/4.1;
delta=1732/50*pi/180; %Angolo di freccia
beta=0*pi/180; %Angolo di Diedro
pitch=0*pi/180; %Pitch angle
num=20; %Numero di profili totale su ciascuna metà;
PARA=1.2; %Parametro che permette di modificare la curvatura centrale (più si avvicna ad 1 pù dietro forma una V

%% Profilo 2D e caratteristiche aerodinamiche
Profile2D=importdata('fastcatch.dat');
Xp = Profile2D.data(:,1).*Chord-Chord;
Zp = Profile2D.data(:,2).*Chord-max(Profile2D.data(:,2).*Chord)/2;
Xp_flip = -(Chord/2.*ones(size(Xp))+Xp)+Chord/2.*ones(size(Xp))-Chord;
Xp_flip = [Xp_flip(numel(Xp_flip)/2+1:end); Xp_flip(1:numel(Xp_flip)/2)];
Zp_flip = [Zp(numel(Xp_flip)/2+1:end); Zp(1:numel(Xp_flip)/2)];

%% Creazione dell Info Box
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
[BoomInfo] = Boom3DShape(BoomInfo,'Info','Create_Stl');
load fastcatch_360.mat
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
X_ini=[9.2000    10   0   75   5];
r0=X_ini(1)*2*pi;
theta=X_ini(2)*pi/180;
D=X_ini(3)*pi/180;
phi=X_ini(4)*pi/180;
Vs=X_ini(5);
[quat,ustart] = HandInitial(r0,theta,D,phi,Vs,BoomInfo);
[V_dx_b,V_sx_b]=InitialConditionPlot(eye(3),quatToAtt(quat),ustart',[0;0;r0],BoomInfo);
theta=20*pi/180;
D=0*pi/180;
Chi=0.85;
%%
tic
[S,Time,Dist,Xm] = StabilityCheck(BoomInfo,theta,D,Chi);
toc
%%
tfin=40;
z0= 1.8; % initial altitude
theta=20*pi/180;
D=0*pi/180;
Chi=0.85;
r0=10*2*pi;
phi=87.5*pi/180;
R=norm(BoomInfo.Aero.P_Finish_Dx);
Vs=r0*R*(1/Chi-1);
options = odeset('Events', @EventsSheronQUAT,'RelTol',1e-4,'AbsTol',1e-6);
Y0=[quat 0 0 r0  ustart(1) ustart(2) ustart(3) 0 0 z0 ]';

tic
[TOUT,YOUT_quat] = ode45(@(t,y)EquationOfMotionsQuaternion(t,y,BoomInfo),[0 tfin],Y0,options); %
toc

[YOUT] = Eul_Quat(YOUT_quat,TOUT);
Energy(TOUT,YOUT,BoomInfo);
PlotTipDxSx(TOUT,YOUT,BoomInfo)
%PlotAeroForce(YOUT,TOUT,BoomInfo)
ChiAvan(BoomInfo,YOUT,TOUT)
FinalReport(YOUT,TOUT);