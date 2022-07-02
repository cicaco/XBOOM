%% Script per creare la Geometria 3D
clear all 
close all
addpath(genpath('BLACKBOX'));
%% Input Data
p_c=10; % numero di profili di "Transizione" nella parte centrale
l=0.3; % lunghezza della pala avente un profilo 2D definito, NON corrisponde alla lunghezza del boomerang
Chord=l/6;
delta= 40*pi/180; %Angolo di freccia
beta=0*pi/180; %Angolo di Diedro
pitch=0*pi/180; %Pitch angle
num=20; %Numero di profili totale su ciascuna metà;
PARA=1.6; %Parametro che permette di modificare la curvatura centrale (più si avvicna ad 1 pù dietro forma una V
% Profile 2D Shape
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
X_ini=[8.7 6.8 7.3 56.6 10.8]*10;
X_ini=[ 10   10   45   70    0];
%     8.9672    0.8426   43.4075   77.6758    3.3413
%     8.1975    3.1834   55.5841   79.5410    3.2427
%     9.4836    1.6689   70.1541   86.5571    4.5907
%     7.5352    4.4579   14.2942   63.1924    4.6925
%     8.6124    2.0826   43.2909   77.4996    3.3397
%     8.9945    4.7260   58.5557   88.7488    3.3306
%     9.0381    1.2379   77.2822   81.7596    4.8276
%     9.5171    4.1131   25.7033   88.9711    4.3507
r0=X_ini(1)*2*pi;
theta=X_ini(2)*pi/180;
D=X_ini(3)*pi/180;
phi=X_ini(4)*pi/180;
Vs=X_ini(5);
Tl_0=eye(3);
R=norm(BoomInfo.Aero.P_Finish_Dx);
Chi=0.70;
Vs=r0*R*(1/Chi-1);
[quat,ustart] = HandInitial(r0,theta,D,phi,Vs,eye(3),BoomInfo);
[V_dx_b,V_sx_b]=InitialConditionPlot(eye(3),quatToAtt(quat),ustart',[0;0;r0],BoomInfo);
[S,Time,Dist,Xm,index] = StabilityCheck(BoomInfo,[3 0 0 50 3],[12 20 90 90 5],100);

tfin=40;
z0= 1.8; % initial altitude

options = odeset('Events', @EventsQUAT,'RelTol',1e-4,'AbsTol',1e-6);
Y0=[quat 0 0 r0  ustart(1) ustart(2) ustart(3) 0 0 z0 ]';

tic
[TOUT,YOUT_quat] = ode45(@(t,y)EquationOfMotionsQuaternion_IND(t,y,BoomInfo,Tl_0),[0 tfin],Y0,options); %
toc

%% Grafici Finali
[YOUT] = Eul_Quat(YOUT_quat,TOUT);
FinalReport(YOUT,TOUT);

Energy(TOUT,YOUT,BoomInfo);
PlotTipDxSx(TOUT,YOUT,BoomInfo,Tl_0)
%PlotAeroForce(YOUT,TOUT,BoomInfo)
ChiAvan(BoomInfo,YOUT,TOUT)
%%
figure()
N=linspace(1,500,500);
plot3(Xm(:,4),Xm(:,2),Xm(:,1),'*r');
hold on
plot3(Xm(index,4),Xm(index,2),Xm(index,1),'ob');
xlabel('Phi');
ylabel('D');
zlabel('r0');
figure()
plot(Xm(:,1),Xm(:,4),'*r');
hold on
plot(Xm(index,1),Xm(index,4),'ob');
figure()
shp=alphaShape(Xm(index,1),Xm(index,4));
plot(shp)
set(gca,'Xlim',[7 12]);
grid on
