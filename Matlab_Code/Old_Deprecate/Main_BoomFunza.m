%% Script per creare la Geometria 3D
clear all 
close all
addpath(genpath('BLACKBOX'));

%% Input Data
Chord=0.0448;
% pr
p_c=20; % numero di profili di "Transizione" nella parte centrale
l=0.3; % lunghezza della pala avente un profilo 2D definito, NON corrisponde alla lunghezza del boomerang
delta=120*pi/180; %Angolo di freccia
beta=0*pi/180; %Angolo di Diedro
pitch=0*pi/180; %Pitch angle
num=20; %Numero di profili totale su ciascuna metà;
PARA=1; %Parametro che permette di modificare la curvatura centrale (più si avvicna ad 1 pù dietro forma una V
% Profile 2D Shape
Profile2D=importdata('Naca0012.dat');
%% Profilo 2D flip e analisi
Xp=-[Profile2D.data(2:67,1) ; fliplr(Profile2D.data(68:end,1)')'].*Chord;
Zp=[Profile2D.data(2:67,2) ; fliplr(Profile2D.data(68:end,2)')'].*Chord;
% Profile2D=importdata('Naca0020.dat');
% Xp=-[0; fliplr(Profile2D(1:65,1)) ; fliplr(Profile2D(66:end,1)')'].*Chord;
% Zp=[0 ;fliplr(Profile2D(1:65,2)) ; fliplr(Profile2D(66:end,2)')'].*Chord;
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
%% Cl Cd ecc
global alpha_t_CD 
global CD_t
data = load('reverse_0012_coeff');
load norm_polar


%sorting alpha
alpha   = data(:,1);
alpha_m = -flip(data(:,1));
alpha_t_CD = [ alpha-180; linspace(-160,-40,6)'; ...
    pol.alpha; linspace(40, 160, 6)'; alpha_m+180] ;

%sorting CD
CD_n      = data(:,3);
CD_m      = flip(data(:,3));
CD_t    = [  CD_n; 1.2*ones(6,1); ...
     pol.CD; 1.2*ones(6,1); CD_m];
 
 data = load('reverse_0012_coeff');
load norm_polar

global alpha_t_CL 
global CL_t

%sorting alpha
alpha   = data(:,1);
alpha_m = -flip(data(:,1));
alpha_t_CL = [ alpha-180; linspace(-160,-40,6)'; ...
    pol.alpha; linspace(40, 160, 6)'; alpha_m+180] ;

%sorting CL
CL_n      = data(:,2);
CL_m    = -flip(data(:,2));
CL_t    = [ CL_n; zeros(6,1); ...
     pol.CL; zeros(6,1); CL_m] ;
 
 data = load('reverse_0012_coeff');
load norm_polar
global alpha_t_CM 
global CM_t


%sorting alpha
alpha   = data(:,1);
alpha_m = -flip(data(:,1));
alpha_t_CM = [ alpha-180; linspace(-160,-40,6)'; ...
    pol.alpha; linspace(40, 160, 6)'; alpha_m+180] ;

%sorting CL
CL_n      = data(:,2);

CM_n    = data(:,5);
CM_n    = CM_n + CL_n*0.5; %0.5 is half the chord
CM_m    = -flip(data(:,5));
CM_t    = [ CM_n; zeros(6,1); ...
     pol.Cm; zeros(6,1); CM_m];
%% Geometry
n=num+p_c;
Dens_i=[1100.*ones(1,n-p_c-1) 600*ones(1,2*p_c-1) 1100.*ones(1,n-p_c-1)];
Dens_i=800.*ones(1,2*n-3);
%Dens_i=[675 500 37.*ones(1,n-p_c-1-2) 250.*ones(1,2*p_c-1)   37.*ones(1,n-p_c-3) 500 675];
%Dens_i=[1100 800.*ones(1,2*n-5) 1100];
%Dens_i=[1000.*ones(1,n-p_c-1) 1000*ones(1,2*p_c-1) 400.*ones(1,n-p_c-1)];

[BoomInfo] = Boom3DShape(BoomInfo,'Info','Density_variation',Dens_i,'Create_Stl');
rho=1000;
BoomInfo.Mecc.I
% BoomInfo.Mecc.I_rho=rho*BoomInfo.Mecc.I;
% BoomInfo.Mecc.m=BoomInfo.Mecc.V*rho;
%%
theta0=0*pi/180;
phi0=-80*pi/180;  % initial inclination of the boomerang -70 / -90 degrees; baseline -85 degrees
psi0=0*pi/180;
r0= 13*2*pi; % initial condition on spin rate 12/15 Hz; baseline 13
z0= 2; % initial altitude

Tl_0=[cos(theta0)*cos(psi0), cos(theta0)*sin(psi0), -sin(theta0)
    -cos(phi0)*sin(psi0)+sin(phi0)*sin(theta0)*cos(psi0), cos(phi0)*cos(psi0)+sin(phi0)*sin(theta0)*sin(psi0), sin(phi0)*cos(theta0)
    sin(phi0)*sin(psi0)+cos(phi0)*sin(theta0)*cos(psi0), -sin(phi0)*cos(psi0)+cos(phi0)*sin(theta0)*sin(psi0), cos(phi0)*cos(theta0)];
% BoomInfo.Mecc.I_rho = 3*BoomInfo.Mecc.I_rho;

% BoomInfo.Mecc.I_rho =10^-4*[ 14 8 0; 8 14 0; 0 0 27];
% BoomInfo.Mecc.m=0.075;
tfin=4;
ustart=Tl_0*[20;0;5];
fileID = fopen('debug.txt','a+');
options = odeset('Events', @Events,'RelTol',1e-4,'AbsTol',1e-6);
Y0=[theta0 phi0 psi0 0  r0  ustart(1) ustart(2) ustart(3) 0 0 z0 ]';

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
chi= YOUT(:,6).*BoomInfo.Pianta.l*cos(BoomInfo.Pianta.freccia)/(vecnorm(YOUT(:,7:9)'))';
V=(vecnorm(YOUT(:,7:9)'))';
figure(20)
plot(TOUT(:),chi);
title('time vs $\chi$','Interpreter','latex');
xlabel('t [s]');
ylabel('$\chi$','Interpreter','latex');
figure(21)
plot(TOUT(:),V);
title('Velocità');