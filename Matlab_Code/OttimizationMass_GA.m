%% Script per creare la Geometria 3D
% Vine considerato il risultato ottenuto tramite il metodo del simplesso e
% ottimizzata la densità del boomerang
clear all
close all
addpath(genpath('BLACKBOX'));
% Input Data
% Input Data

p_c=15; % numero di profili di "Transizione" nella parte centrale
l= 0.280322920216553; % lunghezza della pala avente un profilo 2D definito, NON corrisponde alla lunghezza del boomerang
Chord=l/4.904364778342789;
delta= 45.573878982296918*pi/180; %Angolo di freccia
beta=0*pi/180; %Angolo di Diedro
pitch=0*pi/180; %Pitch angle
num=5; %Numero di profili totale su ciascuna metà;
PARA=1.2; %Parametro che permette di modificare la curvatura centrale (più si avvicna ad 1 pù dietro forma una V

% Profilo 2D e caratteristiche aerodinamiche
Profile2D=importdata('fastcatch.dat');
Xp = Profile2D.data(:,1).*Chord-Chord;
Zp = Profile2D.data(:,2).*Chord-max(Profile2D.data(:,2).*Chord)/2;
Xp_flip = -(Chord/2.*ones(size(Xp))+Xp)+Chord/2.*ones(size(Xp))-Chord;
Xp_flip = [Xp_flip(numel(Xp_flip)/2+1:end); Xp_flip(1:numel(Xp_flip)/2)];
Zp_flip = [Zp(numel(Xp_flip)/2+1:end); Zp(1:numel(Xp_flip)/2)];

% Creazione dell Info Box
BoomInfo.Pianta.l=l;
BoomInfo.Pianta.freccia=delta;
BoomInfo.Pianta.diedro=beta;
BoomInfo.Pianta.pitch=pitch;
BoomInfo.Geom3D.p_c=p_c;
BoomInfo.Geom3D.num=num;
BoomInfo.Geom3D.PARA=PARA;
BoomInfo.Mecc.Dens= 6.327343750000000e+02;
BoomInfo.Profile.Chord=Chord;
BoomInfo.Profile.Xp_sx=fliplr(Xp')';
BoomInfo.Profile.Xp_dx=Xp_flip;
BoomInfo.Profile.Zp_sx=fliplr(Zp')';
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

%
CheckBoomInfo(BoomInfo,'Plot','Text')
%% Initial Condition

theta=0*pi/180;
D=0*pi/180;
Chi=0.85;

[A] = StabilityCheck(BoomInfo,D,theta,Chi);
%%
tfin=5;
z0= 1.8; % initial altitude
r0=13.7914*2*pi;
phi=88.3779*pi/180;
R=norm(BoomInfo.Aero.P_Finish_Dx);
BoomInfo.Mecc.I_rho(2,3)=-BoomInfo.Mecc.I_rho(2,3);
BoomInfo.Mecc.I_rho(3,2)=BoomInfo.Mecc.I_rho(2,3);

Vs=r0*R*(1/Chi-1);
[quat,ustart] = HandInitial(r0,theta,D,phi,Vs,BoomInfo);

options = odeset('Events', @EventsAntiSheronQUAT,'RelTol',1e-4,'AbsTol',1e-6);
Y0=[quat 0 0 r0  ustart(1) ustart(2) ustart(3) 0 0 z0 ]';

tic
[TOUT,YOUT_quat] = ode45(@(t,y)EquationOfMotionsQuaternion(t,y,BoomInfo),[0 tfin],Y0,options); %
toc


[YOUT] = Eul_Quat(YOUT_quat,TOUT);
%Energy(TOUT,YOUT,BoomInfo);
PlotTipDxSx(TOUT,YOUT,BoomInfo)
%PlotAeroForce(YOUT,TOUT,BoomInfo)
%ChiAvan(BoomInfo,YOUT,TOUT)
FinalReport(YOUT,TOUT);
%%
figure(10)
    title('Condizioni di lancio: Forma Ottimizzata Finale','fontsize',12,'interpreter','latex');
    set(gca,'TickLabelInterpreter','latex')
    xlabel('$r_{0}$ [Hz]','fontsize',11,'interpreter','latex');
        ylabel('$\phi$','fontsize',11,'interpreter','latex');