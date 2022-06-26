%% Script per creare la Geometria 3D
clear all 
close all
addpath(genpath('BLACKBOX'));
%% Input Data
Chord=0.05;
p_c=20; % numero di profili di "Transizione" nella parte centrale
l=0.3; % lunghezza della pala avente un profilo 2D definito, NON corrisponde alla lunghezza del boomerang
delta=30*pi/180; %Angolo di freccia
beta=0*pi/180; %Angolo di Diedro
pitch=0*pi/180; %Pitch angle
num=20; %Numero di profili totale su ciascuna metà;
PARA=2.; %Parametro che permette di modificare la curvatura centrale (più si avvicna ad 1 pù dietro forma una V
% Profile 2D Shape
%% Profilo 2D e caratteristiche aerodinamiche
Profile2D=importdata('fastcatch.dat');
Xp = Profile2D.data(:,1).*Chord-Chord;
Zp = Profile2D.data(:,2).*Chord-max(Profile2D.data(:,2).*Chord)/2;
Xp_flip = -(Chord/2.*ones(size(Xp))+Xp)+Chord/2.*ones(size(Xp))-Chord;
Xp_flip = [Xp_flip(numel(Xp_flip)/2+1:end); Xp_flip(1:numel(Xp_flip)/2)];
Zp_flip = [Zp(numel(Xp_flip)/2+1:end); Zp(1:numel(Xp_flip)/2)];
load fastcatch_360.mat
CL_t      = coeff360.CL;
CD_t      = coeff360.CD;
CM_t      = coeff360.CM;
alpha_cl  = coeff360.alpha;
alpha_cd  = coeff360.alpha;
alpha_cm  = coeff360.alpha;
%% Creazione dell Info Box
BoomInfo.Pianta.l=l;
BoomInfo.Pianta.freccia=delta;
BoomInfo.Pianta.diedro=beta;
BoomInfo.Pianta.pitch=pitch;
BoomInfo.Geom3D.p_c=p_c;
BoomInfo.Geom3D.num=num;
BoomInfo.Geom3D.PARA=PARA;
BoomInfo.Profile.Chord=Chord;
BoomInfo.Profile.Xp_sx=Xp;
BoomInfo.Profile.Xp_dx=Xp_flip;
BoomInfo.Profile.Zp_sx=Zp;
BoomInfo.Profile.Zp_dx=Zp_flip;
BoomInfo.Aero.alpha_cl=alpha_cl;
BoomInfo.Aero.alpha_cd=alpha_cd;
BoomInfo.Aero.alpha_cm=alpha_cm;
BoomInfo.Aero.Cl=CL_t;
BoomInfo.Aero.Cd=CD_t;
BoomInfo.Aero.Cm=CM_t;
BoomInfo.Mecc.Dens=650;
BoomInfo.Aero.V_ind=0;
%%
[BoomInfo] = Boom3DShape(BoomInfo,'Info','Create_Stl');
BoomInfo.Mecc.I_rho
BoomInfo.Mecc.m
CheckBoomInfo(BoomInfo,'Plot')
%% Initial condition
fprintf('Starting Genetic Alghoritm...\n');
% x=[r0 theta D phi Vs];
lb=[8 0 0 0 8]*10; %[Hz gradi m/s]
ub=[10 10  90  90 15]*10;
fitnessfcn=@(x) GA_para1(x,BoomInfo);
options = optimoptions('ga', 'MaxStallGenerations', 10, 'MaxGenerations', 10, 'NonlinearConstraintAlgorithm', 'penalty',...
    'PopulationSize', 50, 'PlotFcn', {'gaplotbestindiv', 'gaplotbestf'},...
    'Display', 'iter', 'UseParallel', true, 'UseVectorized', false,'FitnessLimit',4 );
X_ini = ga(fitnessfcn,5,[],[],[],[],lb,ub,[],1:5,options);

%% Initial condition
[PAR] = GA_para1(X_ini,BoomInfo,'Ciao');
X_ini_right=[X_ini(1)/10 X_ini(2)/10 X_ini(3)/10 X_ini(4)/10 X_ini(5)/10]

%%
r0=X_ini(1)*2*pi/10;
theta=X_ini(2)*pi/180/10;
D=X_ini(3)*pi/180/10;
phi=X_ini(4)*pi/180/10;
Vs=X_ini(5)/10;


theta0=0*pi/180;
phi0=0*pi/180;
psi0=0*pi/180;
Tl_0=[cos(theta0)*cos(psi0), cos(theta0)*sin(psi0), -sin(theta0)
    -cos(phi0)*sin(psi0)+sin(phi0)*sin(theta0)*cos(psi0), cos(phi0)*cos(psi0)+sin(phi0)*sin(theta0)*sin(psi0), sin(phi0)*cos(theta0)
    sin(phi0)*sin(psi0)+cos(phi0)*sin(theta0)*cos(psi0), -sin(phi0)*cos(psi0)+cos(phi0)*sin(theta0)*sin(psi0), cos(phi0)*cos(theta0)];
z0= 1.8; % initial altitude
[quat,ustart] = HandInitial(r0,theta,D,phi,Vs,Tl_0,BoomInfo);

tfin=40;


%[V_dx_b,V_sx_b]=InitialConditionPlot(Tl_0,T0,ustart,[0;0;r0],BoomInfo);
[V_dx_b,V_sx_b]=InitialConditionPlot(Tl_0,quatToAtt(quat),ustart',[0;0;r0],BoomInfo);


options = odeset('Events', @EventsQUAT,'RelTol',1e-4,'AbsTol',1e-6);
Y0=[quat 0 0 r0  ustart(1) ustart(2) ustart(3) 0 0 z0 ]';
%%
%%
tic
[TOUT,YOUT_quat] = ode45(@(t,y)EquationOfMotionsQuaternion_IND(t,y,BoomInfo,Tl_0),[0 tfin],Y0,options); %
toc
[YOUT] = Eul_Quat(YOUT_quat,TOUT);
Energy(TOUT,YOUT,BoomInfo)
PlotTipDxSx(TOUT,YOUT,BoomInfo,Tl_0)
%PlotAeroForce(YOUT,TOUT,BoomInfo)
ChiAvan(BoomInfo,YOUT,TOUT)
FinalReport(YOUT,TOUT)
