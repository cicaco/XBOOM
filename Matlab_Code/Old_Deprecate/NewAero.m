%% Script per creare la Geometria 3D
clear all 
close all
addpath(genpath('BLACKBOX'));

%% Input Data
Chord=0.0448;
p_c=20; % numero di profili di "Transizione" nella parte centrale
l=0.3; % lunghezza della pala avente un profilo 2D definito, NON corrisponde alla lunghezza del boomerang
delta=30*pi/180; %Angolo di freccia
beta=0*pi/180; %Angolo di Diedro
pitch=5*pi/180; %Pitch angle
num=20; %Numero di profili totale su ciascuna metà;
PARA=1.6; %Parametro che permette di modificare la curvatura centrale (più si avvicna ad 1 pù dietro forma una V
% Profile 2D Shape
Profile2D=importdata('Naca0012.dat');
%% Profilo 2D flip e analisi
Xp=-[Profile2D.data(2:67,1) ; fliplr(Profile2D.data(68:end,1)')'].*Chord;
Zp=[Profile2D.data(2:67,2) ; fliplr(Profile2D.data(68:end,2)')'].*Chord;
Profile2D=importdata('Naca0025.dat');
Xp=-[0; fliplr(Profile2D(1:65,1)) ; fliplr(Profile2D(66:end,1)')'].*Chord;
Zp=[0 ;fliplr(Profile2D(1:65,2)) ; fliplr(Profile2D(66:end,2)')'].*Chord*1/3;
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
[Cl,Cd,Cm] = AeroCoeff;
BoomInfo.Aero.Cl=Cl;
BoomInfo.Aero.Cd=Cd;
BoomInfo.Aero.Cm=Cm;
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
%% Geometry
[BoomInfo] = Boom3DShape(BoomInfo,'Info','Create_Stl');
% BoomInfo.Mecc.I
BoomInfo.Mecc.m=0.130;
rho=0.130/BoomInfo.Mecc.V;
BoomInfo.Mecc.I_rho=rho*BoomInfo.Mecc.I;
% BoomInfo.Mecc.m=BoomInfo.Mecc.V*rho
BoomInfo.Mecc.CG


%% Provo a costruire una nuova aerodinamica



theta0=10*pi/180;
phi0=45*pi/180;  % initial inclination of the boomerang -70 / -90 degrees; baseline -85 degrees
psi0=180*pi/180;
r0= 13*2*pi; % initial condition on spin rate 12/15 Hz; baseline 13
z0=2; % initial altitude

Tl_0=[cos(theta0)*cos(psi0), cos(theta0)*sin(psi0), -sin(theta0)
    -cos(phi0)*sin(psi0)+sin(phi0)*sin(theta0)*cos(psi0), cos(phi0)*cos(psi0)+sin(phi0)*sin(theta0)*sin(psi0), sin(phi0)*cos(theta0)
    sin(phi0)*sin(psi0)+cos(phi0)*sin(theta0)*cos(psi0), -sin(phi0)*cos(psi0)+cos(phi0)*sin(theta0)*sin(psi0), cos(phi0)*cos(theta0)];
% BoomInfo.Mecc.I_rho = 3*BoomInfo.Mecc.I_rho;

% BoomInfo.Mecc.I_rho =10^-4*[ 14 8 0; 8 14 0; 0 0 27];
% BoomInfo.Mecc.m=0.075;
tfin=0.04;
ustart=(Tl_0*[15;0;0]);
fileID = fopen('debug.txt','a+');
options = odeset('Events', @Events,'RelTol',1e-4,'AbsTol',1e-6);
Y0=[theta0 phi0 psi0 0 0  r0  ustart(1) ustart(2) ustart(3) 0 0 z0 ]';
p=0;
r=r0;
q=0;
Om=[p q r];
ux=ustart(1);
uy=ustart(2);
uz=ustart(3);

%%
%[F_tot,M_tot] = AeroLore(ustart' ,Om,BoomInfo)
R=@(theta0,psi0,phi0) [cos(theta0)*cos(psi0), cos(theta0)*sin(psi0), -sin(theta0)
    -cos(phi0)*sin(psi0)+sin(phi0)*sin(theta0)*cos(psi0), cos(phi0)*cos(psi0)+sin(phi0)*sin(theta0)*sin(psi0), sin(phi0)*cos(theta0)
    sin(phi0)*sin(psi0)+cos(phi0)*sin(theta0)*cos(psi0), -sin(phi0)*cos(psi0)+cos(phi0)*sin(theta0)*sin(psi0), cos(phi0)*cos(theta0)];

[F_tot,M_tot]=AeroDynamics([ux;uy;uz]',[p;q;r]',BoomInfo)
Tl_0'*F_tot
%load('Boom.mat');

%%
[TOUT,YOUT] = ode45(@(t,y)EquationOfMotions(t,y,fileID,BoomInfo),[0 tfin],Y0,options); % 
fclose(fileID);
TOUT(end)
%%
linecolors={'r' 'y' 'c' 'g' 'b' 'k'};
[handles]=plotNy(TOUT(:),YOUT(:,1)*180/pi,1,...
TOUT(:),YOUT(:,2)*180/pi,1,...
TOUT(:),YOUT(:,3)*180/pi,1,...
    TOUT(:),YOUT(:,6),2,... 
    TOUT(:),YOUT(:,4),2,... 
    TOUT(:),YOUT(:,5),2,... 
    TOUT(:),YOUT(:,10),3,...
    TOUT(:),YOUT(:,11),3,... 
    TOUT(:),YOUT(:,12),3,... 
    'YAxisLabels',{ 'Angle [°]' 'Angular Rate [rad/s]' 'Position [m]'},...
    'Linewidth',1,...
    'XLim',[0,TOUT(end)],...
    'XAxisLabel','time[s]',...
    'TitleStr','Lancio Finale',...
    'FontSize',10,...
    'LegendString',{'Theta' 'Phi' 'Psi' 'r' 'p' 'q' 'x' 'y' 'z'});
grid on

%% prova
%geometria
PlotTipDxSx(TOUT,YOUT,BoomInfo)

%%
Time=TOUT(:);
x=YOUT(:,10);
y=YOUT(:,11);
z=YOUT(:,12);
Phi=YOUT(:,2)*180/pi;
Psi=YOUT(:,3)*180/pi;
Theta=YOUT(:,1)*180/pi;
save('T.mat','Time','Theta' ,'Phi' ,'Psi' , 'x' ,'y' ,'z');

%%
