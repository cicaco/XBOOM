%% Script per creare la Geometria 3D
clear all
close all
addpath(genpath('BLACKBOX'));

%% Input Data
Chord=0.045;
p_c=20; % numero di profili di "Transizione" nella parte centrale
l=0.3; % lunghezza della pala avente un profilo 2D definito, NON corrisponde alla lunghezza del boomerang
delta=30*pi/180; %Angolo di freccia
beta=0*pi/180; %Angolo di Diedro
pitch=0*pi/180; %Pitch angle
num=20; %Numero di profili totale su ciascuna metà;
PARA=1.6; %Parametro che permette di modificare la curvatura centrale (più si avvicna ad 1 pù dietro forma una V
% Profile 2D Shape
Profile2D=importdata('Naca0012.dat');
%% Profilo 2D flip e analisi
Xp=-[Profile2D.data(2:67,1) ; fliplr(Profile2D.data(68:end,1)')'].*Chord;
Zp=[Profile2D.data(2:67,2) ; fliplr(Profile2D.data(68:end,2)')'].*Chord;
Profile2D=importdata('Naca0020.dat');
Xp=-[0; fliplr(Profile2D(1:65,1)) ; fliplr(Profile2D(66:end,1)')'].*Chord;
Zp=[0 ;fliplr(Profile2D(1:65,2)) ; fliplr(Profile2D(66:end,2)')'].*Chord;
Xp_flip=-(Chord/2.*ones(size(Xp))+Xp)+Chord/2.*ones(size(Xp))-Chord;
Zp_flip=(Zp);
figure()
plot(Xp,Zp,'*r');
hold on
plot(Xp_flip,Zp_flip,'oc');
axis equal
set(gca,'Xdir','reverse')
[n,~]=size(Xp);
%Clock-wise direction regeneration
Xp_flip=[fliplr(Xp_flip(1:n/2)')';fliplr(Xp_flip(n/2+1:end)')'];
Zp_flip=[fliplr(Zp_flip(1:n/2)')';fliplr(Zp_flip(n/2+1:end)')'];
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
global alpha_t
global CM_t
global CD_t
global CL_t

T_0025=readmatrix('Cl_CD_naca0025.dat');
AoA=T_0025(:,2);
Cl=T_0025(:,3);
Cd=T_0025(:,4);
Cm=T_0025(:,5);

alpha_t=[-fliplr(AoA(2:end)')' ;AoA];
CD_t=[fliplr(Cd(2:end)')'; Cd];
CL_t=[-fliplr(Cl(2:end)')'; Cl];
CM_t=[-fliplr(Cm(2:end)')'; Cm];

figure(2)
plot(alpha_t,CL_t,'r');
hold on
plot(alpha_t,CD_t,'b');
plot(alpha_t,CM_t,'k');

legend('Cl_0025','Cd_0025','Cm_0025','Cl_0012','Cd_0012');
%%
n=num+p_c;

Dens_i=[1100.*ones(1,n-p_c-1) 900*ones(1,2*p_c-1) 1100.*ones(1,n-p_c-1)];
R=1000;
Dens_i=[R.*ones(1,n-p_c-1) R*ones(1,2*p_c-1) R.*ones(1,n-p_c-1)];

%Dens_i=[1500 1500 1000.*ones(1,n-p_c-1-2) 1000.*ones(1,2*p_c-1)   1000.*ones(1,n-p_c-3) 1500 1500];

[BoomInfo] = Boom3DShape(BoomInfo,'Info','Density_variation',Dens_i);

BoomInfo.Mecc.I_rho
BoomInfo.Mecc.m
%% Initial condition
r0= 9.2*2*pi; % initial condition on spin rate 10/15 Hz;

theta=0*pi/180;
D=64*pi/180;

phi=87*pi/180;
Vs=15;
tic
x=[r0 theta D phi Vs];
lb=[7 0 0 0 8]; %[Hz gradi m/s]
ub=[11 10  90  90 15];
fitnessfcn=@(x) GA_para1(x,BoomInfo);
options = optimoptions('ga', 'MaxStallGenerations', 10, 'MaxGenerations', 10, 'NonlinearConstraintAlgorithm', 'penalty',...
    'PopulationSize', 20, 'PlotFcn', {'gaplotbestindiv', 'gaplotbestf'},...
    'Display', 'iter', 'UseParallel', true, 'UseVectorized', false);
x = ga(fitnessfcn,5,[],[],[],[],lb,ub,[],1:5,options);
[PAR] = GA_para1(x,BoomInfo,'Ciao')