%% Script per creare la Geometria 3D
clear all 
close all
addpath(genpath('BLACKBOX'));

%% Input Data
Chord=0.0488;
p_c=20; % numero di profili di "Transizione" nella parte centrale
l=0.3; % lunghezza della pala avente un profilo 2D definito, NON corrisponde alla lunghezza del boomerang
delta=30*pi/180; %Angolo di freccia
beta=0*pi/180; %Angolo di Diedro
pitch=5*pi/180; %Pitch angle
num=20; %Numero di profili totale su ciascuna metà;
PARA=2.; %Parametro che permette di modificare la curvatura centrale (più si avvicna ad 1 pù dietro forma una V
% Profile 2D Shape
Profile2D=importdata('Naca0012.dat');
%% Profilo 2D flip e analisi
Xp=-[Profile2D.data(2:67,1) ; fliplr(Profile2D.data(68:end,1)')'].*Chord;
Zp=[Profile2D.data(2:67,2) ; fliplr(Profile2D.data(68:end,2)')'].*Chord;
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
%% Geometry
[BoomInfo] = Boom3DShape(BoomInfo,'Info');
rho=1000;
BoomInfo.Mecc.I_rho=rho*BoomInfo.Mecc.I;
BoomInfo.Mecc.m=BoomInfo.Mecc.V*rho;
%%
theta0=0*pi/180;
phi0=-80*pi/180;  % initial inclination of the boomerang -70 / -90 degrees; baseline -85 degrees
psi0=0*pi/180;
r0= 13*2*pi; % initial condition on spin rate 12/15 Hz; baseline 13
z0= 1; % initial altitude

Tl_0=[cos(theta0)*cos(psi0), cos(theta0)*sin(psi0), -sin(theta0)
    -cos(phi0)*sin(psi0)+sin(phi0)*sin(theta0)*cos(psi0), cos(phi0)*cos(psi0)+sin(phi0)*sin(theta0)*sin(psi0), sin(phi0)*cos(theta0)
    sin(phi0)*sin(psi0)+cos(phi0)*sin(theta0)*cos(psi0), -sin(phi0)*cos(psi0)+cos(phi0)*sin(theta0)*sin(psi0), cos(phi0)*cos(theta0)];


tfin=20;
ustart=Tl_0*[25;0;0];
fileID = fopen('debug.txt','a+');
options = odeset('Events', @Events,'RelTol',1e-4,'AbsTol',1e-6);
Y0=[theta0 phi0 psi0 0 0 r0  ustart(1) ustart(2) ustart(3) 0 0 z0 ]';

[TOUT,YOUT] = ode45(@(t,y)EquationOfMotions(t,y,fileID,BoomInfo),[0 tfin],Y0,options); % 
fclose(fileID);
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
R=BoomInfo.Pianta.l; %0.30; %m
c=BoomInfo.Profile.Chord; %0.0488; %m
xac=BoomInfo.Aero.P_origin_Sx(1); %va cambiato tra prima e seconda pala (DA FARE)

%sdr PALA j
sigma=BoomInfo.Pianta.freccia +90*pi/180; %120*pi/180;
coning=BoomInfo.Pianta.diedro; %tipo diedro (rot asse x2)
pitch=BoomInfo.Pianta.pitch; %5*pi/180; %pitch della pala (rot asse y3)

%matrice di rotazione da body a blade
Tj=[sin(sigma)*cos(pitch)+cos(sigma)*sin(coning)*sin(pitch), -cos(sigma)*cos(pitch)+sin(sigma)*sin(coning)*sin(pitch), -cos(coning)*sin(pitch)
    cos(sigma)*cos(coning), sin(sigma)*cos(coning), +sin(coning)
    sin(sigma)*sin(pitch)-cos(sigma)*sin(coning)*cos(pitch), -cos(sigma)*sin(pitch)-sin(sigma)*sin(coning)*cos(pitch), cos(coning)*cos(pitch)];

x_tipsx_bodyframe=[xac;0;0]+Tj'*[0;R;0];

%sdr PALA dx
sigma= BoomInfo.Pianta.freccia + 210*pi/180;
coning=BoomInfo.Pianta.diedro; %tipo diedro (rot asse x2)
pitch=BoomInfo.Pianta.pitch; %5*pi/180; %pitch della pala (rot asse y3)
%matrice di rotazione da body a blade
Tj=[sin(sigma)*cos(pitch)+cos(sigma)*sin(coning)*sin(pitch), -cos(sigma)*cos(pitch)+sin(sigma)*sin(coning)*sin(pitch), -cos(coning)*sin(pitch)
    cos(sigma)*cos(coning), sin(sigma)*cos(coning), +sin(coning)
    sin(sigma)*sin(pitch)-cos(sigma)*sin(coning)*cos(pitch), -cos(sigma)*sin(pitch)-sin(sigma)*sin(coning)*cos(pitch), cos(coning)*cos(pitch)];
xac=BoomInfo.Aero.P_origin_Dx(1);
x_tipdx_bodyframe=[xac;0;0]+Tj'*[0;R;0];

x_tipdx=[];
x_tipsx=[];
for i=1:length(TOUT)
theta=(YOUT(i,1));
    phi=(YOUT(i,2));
    psi=(YOUT(i,3));
    T0=[cos(theta)*cos(psi), cos(theta)*sin(psi), -sin(theta)
    -cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi), cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi), sin(phi)*cos(theta)
    sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi), -sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi), cos(phi)*cos(theta)];
x_tipdx=[x_tipdx  [YOUT(i,10); YOUT(i,11); YOUT(i,12)]+T0'*[x_tipdx_bodyframe]];
x_tipsx=[ x_tipsx  [YOUT(i,10); YOUT(i,11); YOUT(i,12)]+T0'*[x_tipsx_bodyframe]];
end
figure()
plot3(YOUT(:,10),YOUT(:,11),YOUT(:,12))
hold on
plot3(x_tipsx(1,:),x_tipsx(2,:),x_tipsx(3,:),'g')
plot3(x_tipdx(1,:),x_tipdx(2,:),x_tipdx(3,:),'r')
legend('CG','tipSX','tipDX')
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;
