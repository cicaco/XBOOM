%% Script per creare la Geometria 3D
clear all
close all
addpath(genpath('BLACKBOX'));

%% Input Data
Chord=0.0488;
p_c=10; % numero di profili di "Transizione" nella parte centrale
l=0.3; % lunghezza della pala avente un profilo 2D definito, NON corrisponde alla lunghezza del boomerang
delta=40*pi/180; %Angolo di freccia
beta=0*pi/180; %Angolo di Diedro
pitch=1*pi/180; %Pitch angle
num=5; %Numero di profili totale su ciascuna metà;
PARA=1.0; %Parametro che permette di modificare la curvatura centrale (più si avvicna ad 1 pù dietro forma una V
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
n=num+p_c;

Dens_i=[1100.*ones(1,n-p_c-1) 900*ones(1,2*p_c-1) 1100.*ones(1,n-p_c-1)];
R=1.0619e+03;
Dens_i=[R.*ones(1,n-p_c-1) R*ones(1,2*p_c-1) R.*ones(1,n-p_c-1)];

%Dens_i=[1500 1500 1000.*ones(1,n-p_c-1-2) 1000.*ones(1,2*p_c-1)   1000.*ones(1,n-p_c-3) 1500 1500];

[BoomInfo] = Boom3DShape(BoomInfo,'Info','Density_variation',Dens_i,'Create_Stl');

BoomInfo.Mecc.I_rho
BoomInfo.Mecc.m
%% Initial condition

% x=[r0 theta D phi Vs];
lb=[8 0 0 0 8]*10; %[Hz gradi m/s]
ub=[12 10  90  90 15]*10;
fitnessfcn=@(x) GA_para1(x,BoomInfo);
options = optimoptions('ga', 'MaxStallGenerations', 10, 'MaxGenerations', 10, 'NonlinearConstraintAlgorithm', 'penalty',...
    'PopulationSize', 150, 'PlotFcn', {'gaplotbestindiv', 'gaplotbestf'},...
    'Display', 'iter', 'UseParallel', true, 'UseVectorized', false,'FitnessLimit',4 );
X_ini = ga(fitnessfcn,5,[],[],[],[],lb,ub,[],1:5,options);
[PAR] = GA_para1(X_ini,BoomInfo,'Ciao');
%%

%% Initial condition
X_ini_right=[X_ini(1)/10 X_ini(2)/10 X_ini(3)/10 X_ini(4)/10 X_ini(5)/10]

%%
r0=X_ini(1)*2*pi/10;
theta=X_ini(2)*pi/180/10;
D=X_ini(3)*pi/180/10;
phi=X_ini(4)*pi/180/10;
Vs=X_ini(5)/10;


theta0=0*pi/180;
phi0=0*pi/180;
psi0=0*pi/180;
Tl_0=[cos(theta0)*cos(psi0), cos(theta0)*sin(psi0), -sin(theta0)
    -cos(phi0)*sin(psi0)+sin(phi0)*sin(theta0)*cos(psi0), cos(phi0)*cos(psi0)+sin(phi0)*sin(theta0)*sin(psi0), sin(phi0)*cos(theta0)
    sin(phi0)*sin(psi0)+cos(phi0)*sin(theta0)*cos(psi0), -sin(phi0)*cos(psi0)+cos(phi0)*sin(theta0)*sin(psi0), cos(phi0)*cos(theta0)];
z0= 1.8; % initial altitude
psi=pi-D;

T0=[cos(theta)*cos(psi), cos(theta)*sin(psi), -sin(theta)
    -cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi), cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi), sin(phi)*cos(theta)
    sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi), -sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi), cos(phi)*cos(theta)];
V_tip=(T0*Tl_0*[Vs*cos(theta)*cos(D);-Vs*cos(theta)*sin(D);Vs*sin(theta)])'; %Velocità della tip nel piano del boomerang
r_mano=[0 0 r0];
P_tip=BoomInfo.Aero.P_Finish_Dx;
ustart=V_tip+cross(r_mano,-P_tip');


eul=[psi theta phi];
quat = eul2quat( eul );

tfin=40;


%[V_dx_b,V_sx_b]=InitialConditionPlot(Tl_0,T0,ustart,[0;0;r0],BoomInfo);


options = odeset('Events', @EventsQUAT,'RelTol',1e-4,'AbsTol',1e-6);
Y0=[quat 0 0 r0  ustart(1) ustart(2) ustart(3) 0 0 z0 ]';
%%
[TOUT,YOUT_quat] = ode45(@(t,y)EquationOfMotionsQuaternion(t,y,BoomInfo,Tl_0),[0 tfin],Y0,options); %
%[TOUT,YOUT_quat] = ode45(@(t,y)TestEquationOfMotionsQuaternion(t,y,BoomInfo,Tl_0),[0 tfin],Y0,options); %
if not( PAR(2)==TOUT(end) && PAR(1)==norm(YOUT_quat(end,11:13)))
    fprintf('Errore \n');
end
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
%Calcolo Energia Cinetica e Potenziale

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
