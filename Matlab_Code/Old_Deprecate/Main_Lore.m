%% Script per creare la Geometria 3D
clear all 
close all
addpath(genpath('BLACKBOX'));
Chord=0.04;
p_c=20; % numero di profili di "Transizione" nella parte centrale
l=0.3; % lunghezza della pala avente un profilo 2D definito, NON corrisponde alla lunghezza del boomerang
delta=30*pi/180; %Angolo di freccia
beta=0*pi/180; %Angolo di Diedro
pitch=10*pi/180; %Pitch angle
num=20; %Numero di profili totale su ciascuna metà;
PARA=2.; %Parametro che permette di modificare la curvatura centrale (più si avvicna ad 1 pù dietro forma una V


%% Profilo 2D flip e analisi
Profile2D=importdata('Naca0012.dat');
%% Creazione dell Info Box
BoomInfo.Pianta.freccia=delta;
BoomInfo.Pianta.diedro=beta;
BoomInfo.Pianta.pitch=pitch;
BoomInfo.Geom3D.p_c=p_c;
BoomInfo.Geom3D.num=num;
BoomInfo.Profile.Chord=Chord;
BoomInfo.Geom3D.PARA=PARA;

Xp=-[Profile2D.data(2:67,1) ; fliplr(Profile2D.data(68:end,1)')'].*Chord;
Zp=[Profile2D.data(2:67,2) ; fliplr(Profile2D.data(68:end,2)')'].*Chord;
Xp_flip=-(Chord/2.*ones(size(Xp))+Xp)+Chord/2.*ones(size(Xp))-Chord;
Zp_flip=(Zp);
[n,~]=size(Xp);
%Clock-wise direction regeneration
Xp_flip=[fliplr(Xp_flip(1:n/2)')';fliplr(Xp_flip(n/2+1:end)')'];
Zp_flip=[fliplr(Zp_flip(1:n/2)')';fliplr(Zp_flip(n/2+1:end)')'];
BoomInfo.Profile.Xp_dx=Xp;
BoomInfo.Profile.Xp_sx=Xp_flip;
BoomInfo.Profile.Zp_dx=Zp;
BoomInfo.Profile.Zp_sx=Zp_flip;

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
l_i=linspace(0.1,0.3,10);
for i=1:10
       BoomInfo.Pianta.l=l_i(i);


    [BoomInfo] = Boom3DShape(BoomInfo,'Info');
    
    rho=1000; %Materiale plastico generico
    BoomInfo.Mecc.I_rho=rho*BoomInfo.Mecc.I;
    BoomInfo.Mecc.m=BoomInfo.Mecc.V*rho;
    m=BoomInfo.Mecc.V*rho
    fileID = fopen('debug.txt','a+');
    options = odeset('Events', @Events,'RelTol',1e-4,'AbsTol',1e-6);
    Y0=[theta0 phi0 psi0 0 0 r0  ustart(1) ustart(2) ustart(3) 0 0 z0 ]';
    
    [TOUT,YOUT] = ode45(@(t,y)EquationOfMotions(t,y,fileID,BoomInfo),[0 tfin],Y0,options); %
    fclose(fileID);
    TOUT(end)
end
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