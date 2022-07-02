%% Script per creare la Geometria 3D
clear all 
close all
addpath(genpath('BLACKBOX'));
%% Input Data
Chord=0.0488;
p_c=20; % numero di profili di "Transizione" nella parte centrale
l=0.3; % lunghezza della pala avente un profilo 2D definito, NON corrisponde alla lunghezza del boomerang
delta=30*pi/180; %Angolo di freccia
beta=1*pi/180; %Angolo di Diedro
pitch=4*pi/180; %Pitch angle
num=20; %Numero di profili totale su ciascuna metà;
% Profile 2D Shape
PARA=2.0;
Profile2D=importdata('Naca0012.dat');
Xp=-[Profile2D.data(2:67,1) ; fliplr(Profile2D.data(68:end,1)')'].*Chord;
Zp=[Profile2D.data(2:67,2) ; fliplr(Profile2D.data(68:end,2)')'].*Chord;
Xp_flip=-(Chord/2.*ones(size(Xp))+Xp)+Chord/2.*ones(size(Xp))-Chord;
Zp_flip=(Zp);
[n,~]=size(Xp);
%Clock-wise direction regeneration
Xp_flip=[fliplr(Xp_flip(1:n/2)')';fliplr(Xp_flip(n/2+1:end)')'];
Zp_flip=[fliplr(Zp_flip(1:n/2)')';fliplr(Zp_flip(n/2+1:end)')'];
alpha_cd=[-180 -170 -160 -130 -90 -50 -20 -10 10 20 50 90 130 160 170 180];
CD_t=[0.01 0.01 0.41 1.25 1.7 1.25 0.41 0.01 0.01 0.41 1.25 1.7 1.25 0.41 0.01 0.01];
alpha_cl=[-180 -170 -130 -60 -12 12 60 130 170 180];
CL_t=[0 0.7 0.7 -1.0 -1.0 1.0 1.0 -0.7 -0.7 0];
alpha_cm=[-180 -150 -90 0 90 150 180];
CM_t=[0 0.4 0.4 0 -0.4 -0.4 0];
BoomInfo.Pianta.l=l;
BoomInfo.Pianta.freccia=delta;
BoomInfo.Pianta.diedro=beta;
BoomInfo.Pianta.pitch=pitch;
BoomInfo.Geom3D.p_c=p_c;
BoomInfo.Geom3D.num=num;
BoomInfo.Geom3D.PARA=PARA;
BoomInfo.Mecc.Dens=1000;
BoomInfo.Profile.Chord=Chord;
BoomInfo.Profile.Xp_dx=Xp;
BoomInfo.Profile.Xp_sx=Xp_flip;
BoomInfo.Profile.Zp_dx=Zp;
BoomInfo.Profile.Zp_sx=Zp_flip;
%BoomInfo.Geom3D.PARA=PARA;

BoomInfo.Aero.alpha_cl=alpha_cl;
BoomInfo.Aero.alpha_cd=alpha_cd;
BoomInfo.Aero.alpha_cm=alpha_cm;
BoomInfo.Aero.Cl=CL_t;
BoomInfo.Aero.Cd=CD_t;
BoomInfo.Aero.Cm=CM_t;
%%
[BoomInfo] = Boom3DShape(BoomInfo,'Info','Create_Stl','FileName','Naca0012_boom');
BoomInfo.Mecc.I_rho
BoomInfo.Mecc.m
CheckBoomInfo(BoomInfo,'Plot')
%% Initial condition
fprintf('Starting Genetic Alghoritm...\n');
% x=[r0 theta D phi Vs];
lb=[3 0 0 0 8]*10; %[Hz gradi m/s]
ub=[8 20  90  90 15]*10;
fitnessfcn=@(x) GA_para1(x,BoomInfo);
options = optimoptions('ga', 'MaxStallGenerations', 10, 'MaxGenerations', 10, 'NonlinearConstraintAlgorithm', 'penalty',...
    'PopulationSize', 20, 'PlotFcn', {'gaplotbestindiv', 'gaplotbestf'},...
    'Display', 'iter', 'UseParallel', true, 'UseVectorized', false,'FitnessLimit',4 );
X_ini = ga(fitnessfcn,5,[],[],[],[],lb,ub,[],1:5,options);

%% Initial condition
[PAR,t_min,r_max] = GA_para1(X_ini,BoomInfo,'Ciao')
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
Energy(TOUT,YOUT,BoomInfo);
PlotTipDxSx(TOUT,YOUT,BoomInfo,Tl_0)
%PlotAeroForce(YOUT,TOUT,BoomInfo)
ChiAvan(BoomInfo,YOUT,TOUT)
FinalReport(YOUT,TOUT)