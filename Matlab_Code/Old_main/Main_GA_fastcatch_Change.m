%% Script per creare la Geometria 3D
clear all 
close all
addpath(genpath('BLACKBOX'));
%% Input Data
p_c=6; % numero di profili di "Transizione" nella parte centrale
beta=0*pi/180; %Angolo di Diedro
pitch=0*pi/180; %Pitch angle
num=20; %Numero di profili totale su ciascuna metà;
PARA=2.0; %Parametro che permette di modificare la curvatura centrale (più si avvicna ad 1 pù dietro forma una V
% Profile 2D Shape
%% Profilo 2D e caratteristiche aerodinamiche
load fastcatch_360.mat
CL_t      = coeff360.CL;
CD_t      = coeff360.CD;
CM_t      = coeff360.CM;
alpha_cl  = coeff360.alpha;
alpha_cd  = coeff360.alpha;
alpha_cm  = coeff360.alpha;
%% Creazione dell Info Box
BoomInfo.Pianta.diedro=beta;
BoomInfo.Pianta.pitch=pitch;
BoomInfo.Geom3D.p_c=p_c;
BoomInfo.Geom3D.num=num;
BoomInfo.Geom3D.PARA=PARA;
BoomInfo.Aero.alpha_cl=alpha_cl;
BoomInfo.Aero.alpha_cd=alpha_cd;
BoomInfo.Aero.alpha_cm=alpha_cm;
BoomInfo.Aero.Cl=CL_t;
BoomInfo.Aero.Cd=CD_t;
BoomInfo.Aero.Cm=CM_t;
BoomInfo.Mecc.Dens=650;
BoomInfo.Aero.V_ind=0;
%%
l=0.3; % lunghezza della pala avente un profilo 2D definito, NON corrisponde alla lunghezza del boomerang
Chord=0.06;
delta=50*pi/180; %Angolo di freccia

li=linspace(0.2,0.4,10);
Di=linspace(30,60,10);
Dist=zeros(10,10);
Tm=zeros(10,10);
M=zeros(10,10);
Rm=zeros(10,10);
Xm=zeros(10,10,5);
In=zeros(3,3,100);
for i=1:10
    BoomInfo.Pianta.freccia=Di(i)*pi/180;
    for j=1:10
        li(j)
        Di(i)
        
        BoomInfo.Pianta.l=li(j);
        BoomInfo.Profile.Chord=li(j)/6;
        Profile2D=importdata('fastcatch.dat');
        Xp = Profile2D.data(:,1).*Chord-Chord;
        Zp = Profile2D.data(:,2).*Chord-max(Profile2D.data(:,2).*Chord)/2;
        Xp_flip = -(Chord/2.*ones(size(Xp))+Xp)+Chord/2.*ones(size(Xp))-Chord;
        Xp_flip = [Xp_flip(numel(Xp_flip)/2+1:end); Xp_flip(1:numel(Xp_flip)/2)];
        Zp_flip = [Zp(numel(Xp_flip)/2+1:end); Zp(1:numel(Xp_flip)/2)];
        BoomInfo.Profile.Xp_sx=Xp;
        BoomInfo.Profile.Xp_dx=Xp_flip;
        BoomInfo.Profile.Zp_sx=Zp;
        BoomInfo.Profile.Zp_dx=Zp_flip;
        [BoomInfo] = Boom3DShape(BoomInfo,'Info');
        M(i,j)=BoomInfo.Mecc.m;
        In(:,:,10*(i-1)+j)=BoomInfo.Mecc.I_rho;
        %CheckBoomInfo(BoomInfo,'Plot')
        % Initial condition
        fprintf('Starting Genetic Alghoritm...\n');
        % x=[r0 theta D phi Vs];
        lb=[3 0 0 0 8]*10; %[Hz gradi m/s]
        ub=[10 10  90  90 15]*10;
        fitnessfcn=@(x) GA_para1(x,BoomInfo);
        options = optimoptions('ga', 'MaxStallGenerations', 10, 'MaxGenerations', 10, 'NonlinearConstraintAlgorithm', 'penalty',...
            'PopulationSize', 20, 'PlotFcn', {'gaplotbestindiv', 'gaplotbestf'},...
            'Display', 'iter', 'UseParallel', true, 'UseVectorized', false,'FitnessLimit',4 );
        X_ini = ga(fitnessfcn,5,[],[],[],[],lb,ub,[],1:5,options);
        %% Initial condition
        [PAR,Tmin,Rmax] = GA_para1(X_ini,BoomInfo,'Ciao');
        X_ini_right=[X_ini(1)/10 X_ini(2)/10 X_ini(3)/10 X_ini(4)/10 X_ini(5)/10];
        Dist(i,j)=PAR(1);
        Tm(i,j)=Tmin;
        Rm(i,j)=Rmax;
        Xm(i,j,:)=reshape(X_ini_right,[1 1 5]);
    end
end
