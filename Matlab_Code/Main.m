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
%% Traiectory
theta0=0*pi/180;
phi0=-80*pi/180;  % initial inclination of the boomerang -70 / -90 degrees; baseline -85 degrees
psi0=0*pi/180;
r0= 17*2*pi; % initial condition on spin rate 12/15 Hz; baseline 13
z0= 1; % initial altitude

Tl_0=[cos(theta0)*cos(psi0), cos(theta0)*sin(psi0), -sin(theta0)
    -cos(phi0)*sin(psi0)+sin(phi0)*sin(theta0)*cos(psi0), cos(phi0)*cos(psi0)+sin(phi0)*sin(theta0)*sin(psi0), sin(phi0)*cos(theta0)
    sin(phi0)*sin(psi0)+cos(phi0)*sin(theta0)*cos(psi0), -sin(phi0)*cos(psi0)+cos(phi0)*sin(theta0)*sin(psi0), cos(phi0)*cos(theta0)];


tfin=20;
ustart=Tl_0*[25;0;0];
fileID = fopen('debug.txt','a+');
options = odeset('Events', @Events,'RelTol',1e-4,'AbsTol',1e-6);
Y0=[theta0 phi0 psi0 0 0 r0  ustart(1) ustart(2) ustart(3) 0 0 z0 ]';

BoomInfo.Mecc.I_rho = 3*BoomInfo.Mecc.I_rho;

[TOUT,YOUT] = ode45(@(t,y)EquationOfMotions(t,y,fileID,BoomInfo),[0 tfin],Y0,options); % 
fclose(fileID);
%% Plot
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

%% grafico
PlotTipDxSx(TOUT,YOUT,BoomInfo)
%% grafico rapporto di avanzamento e velocità
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
%% MASSIMA DISTANZA E TEMPO DI RITORNO
% esclusione tocco terra
if YOUT(end,12) <= 0.01
    % messo a inf -> caso peggiore che sarà scartato dall'ottimizzatore
    % perchè cade a terra
    t_min = inf;
    r_max = -inf;
% il boomerang ritorna e sono in grado di riprenderlo
elseif  TOUT(end)>2 && (sqrt(YOUT(end,10)^2+YOUT(end,11)^2+YOUT(end,12)^2)<=3)
    t_min = TOUT(end);
    r_max = max(sqrt(YOUT(:,10).^2+YOUT(:,11).^2+YOUT(:,12).^2));
end

disp('-------------------------------------------------------------------')
fprintf('Tempo di ritorno calcolato: %.5f s\n', t_min)
fprintf('Massima distanza percorsa: %.5f m\n', r_max)
disp('-------------------------------------------------------------------')