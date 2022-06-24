%% Script per creare la Geometria 3D
clear all 
close all
addpath(genpath('BLACKBOX'));
%% Input Data
Chord=0.0488;
p_c=20; % numero di profili di "Transizione" nella parte centrale
l=0.3; % lunghezza della pala avente un profilo 2D definito, NON corrisponde alla lunghezza del boomerang
delta=30*pi/180; %Angolo di freccia
beta=0.5*pi/180; %Angolo di Diedro
pitch=4*pi/180; %Pitch angle
num=20; %Numero di profili totale su ciascuna metà;
PARA=2.; %Parametro che permette di modificare la curvatura centrale (più si avvicna ad 1 pù dietro forma una V
% Profile 2D Shape
%% Profilo 2D flip e analisi
Profile2D=importdata('Naca0018.dat');
Xp=-[0; fliplr(Profile2D(1:65,1)')' ; fliplr(Profile2D(66:end,1)')'].*Chord;
Zp=[0 ;fliplr(Profile2D(1:65,2)')' ; fliplr(Profile2D(66:end,2)')'].*Chord;
Xp_flip=-(Chord/2.*ones(size(Xp))+Xp)+Chord/2.*ones(size(Xp))-Chord;
Zp_flip=(Zp);
[n,~]=size(Xp);
%Clock-wise direction regeneration
Xp_flip=[fliplr(Xp_flip(1:n/2)')';fliplr(Xp_flip(n/2+1:end)')'];
Zp_flip=[fliplr(Zp_flip(1:n/2)')';fliplr(Zp_flip(n/2+1:end)')'];
figure()
plot(Xp,Zp,'r');
hold on
plot(Xp_flip,Zp_flip,'b');
axis equal
set(gca,'Xdir','reverse')
[n,~]=size(Xp);
%Clock-wise direction regeneration

%% Creazione dell Info Box

alpha_cd=[-180 -170 -160 -130 -90 -50 -20 -10 10 20 50 90 130 160 170 180];
CD_t=[0.01 0.01 0.41 1.25 1.7 1.25 0.41 0.01 0.01 0.41 1.25 1.7 1.25 0.41 0.01 0.01];

alpha_cm=[-180 -150 -90 0 90 150 180];
CM_t=[0 0.4 0.4 0 -0.4 -0.4 0];


alpha_cl=[-180 -170 -130 -60 -12 12 60 130 170 180];
CL_t=[0 0.7 0.7 -1.0 -1.0 1.0 1.0 -0.7 -0.7 0];
for i=1:10
    Cl_simo(i)=CL(alpha_cl(i)*pi/180);
    
end
for i=1:16
     Cd_simo(i)=CD(alpha_cd(i)*pi/180);
end
for i=1:7
     Cm_simo(i)=CM(alpha_cm(i)*pi/180);
end
% norm(CL_t-Cl_simo)
% norm(CD_t-Cd_simo)
% norm(CM_t-Cm_simo)
%%
figure()
plot(alpha_cl,CL_t,'--r');
hold on
plot(alpha_cd,CD_t,'.-b');
plot(alpha_cm,CM_t,'k');
grid on
legend('CL','CD','CM');
BoomInfo.Aero.alpha_cl=alpha_cl;
BoomInfo.Aero.alpha_cd=alpha_cd;
BoomInfo.Aero.alpha_cm=alpha_cm;
BoomInfo.Aero.Cl=CL_t;
BoomInfo.Aero.Cd=CD_t;
BoomInfo.Aero.Cm=CM_t;
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
n=num+p_c;
R=650;
Dens_i=[R.*ones(1,n-p_c-1) R*ones(1,2*p_c-1) R.*ones(1,n-p_c-1)];
[BoomInfo] = Boom3DShape(BoomInfo,'Info','Density_variation',Dens_i);


BoomInfo.Mecc.I_rho
BoomInfo.Mecc.m
%% Initial condition
theta0=0.0*pi/180;
phi0=90*pi/180;
psi0=180*pi/180;

Tl_0=[cos(theta0)*cos(psi0), cos(theta0)*sin(psi0), -sin(theta0)
    -cos(phi0)*sin(psi0)+sin(phi0)*sin(theta0)*cos(psi0), cos(phi0)*cos(psi0)+sin(phi0)*sin(theta0)*sin(psi0), sin(phi0)*cos(theta0)
    sin(phi0)*sin(psi0)+cos(phi0)*sin(theta0)*cos(psi0), -sin(phi0)*cos(psi0)+cos(phi0)*sin(theta0)*sin(psi0), cos(phi0)*cos(theta0)];

%% Traiectory
theta=0*pi/180;
phi=0*pi/180; 
psi=-60*pi/180;
r0= 14*2*pi; % initial condition on spin rate 10/15 Hz;
z0= 2; % initial altitude

T0=[cos(theta)*cos(psi), cos(theta)*sin(psi), -sin(theta)
    -cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi), cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi), sin(phi)*cos(theta)
    sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi), -sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi), cos(phi)*cos(theta)];



ustart=T0*Tl_0*[25*cos(5*pi/180);0;25*sin(5*pi/180)];

eul=[psi theta phi];
quat = eul2quat( eul );

tfin=40;


%[V_dx_b,V_sx_b]=InitialConditionPlot(Tl_0,T0,ustart,[0;0;r0],BoomInfo);

options = odeset('Events', @EventsQUAT,'RelTol',1e-4,'AbsTol',1e-6);
Y0=[quat 0 0 r0  ustart(1) ustart(2) ustart(3) 0 0 z0 ]';
%%
tic
[TOUT,YOUT_quat] = ode15s(@(t,y)EquationOfMotionsQuaternion(t,y,BoomInfo,Tl_0),[0 tfin],Y0,options); %
toc
euler=[];
for i=1:numel(TOUT)
euli = quatToEuler(YOUT_quat(i,1:4) );
euler(i,:)=[euli(2) euli(1) euli(3)];
end
YOUT=[unwrap(euler) YOUT_quat(:,5:end)];
YOUT_Q=YOUT;
save('YOUT_QUAT','YOUT_Q')
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
