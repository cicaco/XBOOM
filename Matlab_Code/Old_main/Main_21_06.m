%% Script per creare la Geometria 3D
clear all
close all
addpath(genpath('BLACKBOX'));

%% Input Data
Chord=0.04;
p_c=5; % numero di profili di "Transizione" nella parte centrale
l=0.3; % lunghezza della pala avente un profilo 2D definito, NON corrisponde alla lunghezza del boomerang
delta=40*pi/180; %Angolo di freccia
beta=0*pi/180; %Angolo di Diedro
pitch=3*pi/180; %Pitch angle
num=2; %Numero di profili totale su ciascuna metà;
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
[n,~]=size(Xp);

%Clock-wise direction regeneration
Xp_flip=[fliplr(Xp_flip(1:n/2)')';fliplr(Xp_flip(n/2+1:end)')'];
Zp_flip=[fliplr(Zp_flip(1:n/2)')';fliplr(Zp_flip(n/2+1:end)')'];
%% Creazione dell Info Box
BoomInfo.Pianta.diedro=beta;
BoomInfo.Pianta.pitch=pitch;
BoomInfo.Geom3D.p_c=p_c;
BoomInfo.Geom3D.num=num;
BoomInfo.Profile.Xp_dx=Xp;
BoomInfo.Profile.Xp_sx=Xp_flip;
BoomInfo.Profile.Zp_dx=Zp;
BoomInfo.Profile.Zp_sx=Zp_flip;
%% Geometry
%[r0 theta D phi Vs Chord PARA Delta l];
lb=[7   0 0   0  8   1 20 0.3*10]*10; %[Hz gradi m/s]
ub=[11 10 90  90 13  2 60 0.5*10]*10;
fitnessfcn=@(x) GA_para2(x,BoomInfo);
options = optimoptions('ga', 'MaxStallGenerations', 10, 'MaxGenerations', 15, 'NonlinearConstraintAlgorithm', 'penalty',...
    'PopulationSize', 20, 'PlotFcn', {'gaplotbestindiv', 'gaplotbestf'},...
    'Display', 'iter', 'UseParallel', true, 'UseVectorized', false);
x = ga(fitnessfcn,9,[],[],[],[],lb,ub,[],1:9,options);
[PAR] = GA_para2(x,BoomInfo,'Ciao')