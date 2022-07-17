%% Script per creare la Geometria 3D
% Vengono calcolate le condizioni di lancio utilizzando il GA dati i 2
% parametri delle condizioni iniziali: r0 e Phi, e tre parametri fissi,
% chi,theta e D
clear all 
close all
addpath(genpath('BLACKBOX'));
%% Input Data
p_c=15; % numero di profili di "Transizione" nella parte centrale
l= 0.2930; % lunghezza della pala avente un profilo 2D definito, NON corrisponde alla lunghezza del boomerang
Chord=l/4.62;
delta= 40*pi/180; %Angolo di freccia
beta=0*pi/180; %Angolo di Diedro
pitch=0*pi/180; %Pitch angle
num=5; %Numero di profili totale su ciascuna metà;
PARA=1.2; %Parametro che permette di modificare la curvatura centrale (più si avvicna ad 1 pù dietro forma una V
l=0.3;
c=0.05;
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
BoomInfo.Profile.Chord=Chord;
BoomInfo.Profile.Xp_sx=Xp;
BoomInfo.Profile.Xp_dx=fliplr(Xp_flip')';
BoomInfo.Profile.Zp_sx=Zp;
BoomInfo.Profile.Zp_dx=fliplr(Zp_flip')';

BoomInfo.Mecc.Dens=650;
BoomInfo.Aero.V_ind=0;
%%
[BoomInfo] = Boom3DShape(BoomInfo,'Info','Create_Stl','FileName','FastCatch_boom');
BoomInfo.Mecc.I_rho
BoomInfo.Mecc.m
%%
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
CheckBoomInfo(BoomInfo,'Plot')
%% Initial condition
theta=0*pi/180;
D=0*pi/180;
Chi=0.85;
%%
P_tot=BoomInfo.Geom3D.Profile;
save('P3D.mat','P_tot');
 [S] = StabilityCheck(BoomInfo,D,theta,Chi);