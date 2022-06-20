%% Script per creare la Geometria 3D
clear all
close all
addpath(genpath('BLACKBOX'));

%% Input Data
Chord=0.045;
p_c=20; % numero di profili di "Transizione" nella parte centrale
l=0.3; % lunghezza della pala avente un profilo 2D definito, NON corrisponde alla lunghezza del boomerang
delta=40*pi/180; %Angolo di freccia
beta=0*pi/180; %Angolo di Diedro
pitch=3*pi/180; %Pitch angle
num=20; %Numero di profili totale su ciascuna metà;
PARA=1.6; %Parametro che permette di modificare la curvatura centrale (più si avvicna ad 1 pù dietro forma una V
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
%%
[Cl,Cd,Cm] = AeroCoeff;
BoomInfo.Aero.Cl=Cl;
BoomInfo.Aero.Cd=Cd;
BoomInfo.Aero.Cm=Cm;
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
n=num+p_c;

Dens_i=[1100.*ones(1,n-p_c-1) 900*ones(1,2*p_c-1) 1100.*ones(1,n-p_c-1)];
R=1000;
Dens_i=[R.*ones(1,n-p_c-1) R*ones(1,2*p_c-1) R.*ones(1,n-p_c-1)];

%Dens_i=[1500 1500 1000.*ones(1,n-p_c-1-2) 1000.*ones(1,2*p_c-1)   1000.*ones(1,n-p_c-3) 1500 1500];

[BoomInfo] = Boom3DShape(BoomInfo,'Info','Density_variation',Dens_i);

BoomInfo.Mecc.I_rho
BoomInfo.Mecc.m
%rho*BoomInfo.Mecc.I
%BoomInfo.Mecc.I_rho=rho*BoomInfo.Mecc.I;
%BoomInfo.Mecc.m=BoomInfo.Mecc.V*1000;
P_tot=BoomInfo.Geom3D.Profile;
save('P3D.mat','P_tot');
%% Initial condition
theta0=0*pi/180;
phi0=0*pi/180;
psi0=0*pi/180;
Tl_0=[cos(theta0)*cos(psi0), cos(theta0)*sin(psi0), -sin(theta0)
    -cos(phi0)*sin(psi0)+sin(phi0)*sin(theta0)*cos(psi0), cos(phi0)*cos(psi0)+sin(phi0)*sin(theta0)*sin(psi0), sin(phi0)*cos(theta0)
    sin(phi0)*sin(psi0)+cos(phi0)*sin(theta0)*cos(psi0), -sin(phi0)*cos(psi0)+cos(phi0)*sin(theta0)*sin(psi0), cos(phi0)*cos(theta0)];
r0= 8.7*2*pi; % initial condition on spin rate 10/15 Hz;
z0= 1.6; % initial altitude
theta=0*pi/180;
D=64*pi/180;
psi=pi-D;

phi=86*pi/180;

T0=[cos(theta)*cos(psi), cos(theta)*sin(psi), -sin(theta)
    -cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi), cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi), sin(phi)*cos(theta)
    sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi), -sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi), cos(phi)*cos(theta)];

ustart=T0*Tl_0*[28*cos(theta)*cos(D);-28*cos(theta)*sin(D);28*sin(theta)];
eul=[psi theta phi];
quat = eul2quat( eul );
quat=[quat(2:4) quat(1)];
tfin=12;
T0_e = quatToAtt( quat );

%[V_dx_b,V_sx_b]=InitialConditionPlot(Tl_0,T0,ustart,[0;0;r0],BoomInfo);

fileID = fopen('debug.txt','a+');
options = odeset('Events', @EventsQUAT,'RelTol',1e-4,'AbsTol',1e-6);
Y0=[quat 0 0 r0  ustart(1) ustart(2) ustart(3) 0 0 z0 ]';
%%
[TOUT,YOUT_quat] = ode15s(@(t,y)EquationOfMotionsQuaternion(t,y,fileID,BoomInfo,Tl_0),[0 tfin],Y0,options); %
%fclose(fileID);
euler=[];
for i=1:numel(TOUT)
euli = quatToEuler(YOUT_quat(i,1:4) )*180/pi;
euler(i,:)=[euli(2) euli(3) euli(1)];
end
YOUT=[euler YOUT_quat(:,5:end)];
%%
% figure()
% plot(phi_vect*180/pi,Dist,'b','linewidth',1.2)
% hold on
% grid on
% xlabel('$\Phi$','fontsize',11,'interpreter','latex');
% set(gca,'TickLabelInterpreter','latex')
% ylabel('DIST','fontsize',11,'interpreter','latex');
% title('Boomerang Traiectory Function of $\Phi$','fontsize',12,'interpreter','latex');
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
PlotTipDxSx(TOUT,YOUT,BoomInfo,Tl_0)
PlotAeroForce(YOUT,TOUT,BoomInfo)
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
else
    t_min = TOUT(end);
    r_max = max(sqrt(YOUT(:,10).^2+YOUT(:,11).^2+YOUT(:,12).^2));
end

disp('-------------------------------------------------------------------')
fprintf('Tempo di ritorno calcolato: %.5f s\n', t_min)
fprintf('Massima distanza percorsa: %.5f m\n', r_max)
disp('-------------------------------------------------------------------')
%%
Time=TOUT(:);
x=YOUT(:,10);
y=YOUT(:,11);
z=YOUT(:,12);
Phi=YOUT(:,2)*180/pi;
Psi=YOUT(:,3)*180/pi;
Theta=YOUT(:,1)*180/pi;
save('T.mat','Time','Theta' ,'Phi' ,'Psi' , 'x' ,'y' ,'z');
