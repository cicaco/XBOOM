%% Script per creare la Geometria 3D
clear all
close all
addpath(genpath('BLACKBOX'));

%% Input Data
Chord=0.04;
p_c=10; % numero di profili di "Transizione" nella parte centrale
l=0.2; % lunghezza della pala avente un profilo 2D definito, NON corrisponde alla lunghezza del boomerang
delta=40*pi/180; %Angolo di freccia
beta=1*pi/180; %Angolo di Diedro
pitch=1*pi/180; %Pitch angle
num=5; %Numero di profili totale su ciascuna metà;
PARA=2.0; %Parametro che permette di modificare la curvatura centrale (più si avvicna ad 1 pù dietro forma una V
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

T_0025=readmatrix('Cl_CD_naca0025.dat');
AoA=T_0025(:,2);
Cl=T_0025(:,3);
Cd=T_0025(:,4);
Cm=T_0025(:,5);

alpha_t=[-fliplr(AoA(2:end)')'*pi/180 ;AoA*pi/180];
CD_t=[fliplr(Cd(2:end)')'; Cd];
CL_t=[-fliplr(Cl(2:end)')'; Cl];
CM_t=[-fliplr(Cm(2:end)')'; Cm]; 
BoomInfo.Aero.alpha_t=alpha_t;
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
%%
n=num+p_c;

Dens_i=[1100.*ones(1,n-p_c-1) 900*ones(1,2*p_c-1) 1100.*ones(1,n-p_c-1)];
R=1.0619e+03;
R=650;
Dens_i=[R.*ones(1,n-p_c-1) R*ones(1,2*p_c-1) R.*ones(1,n-p_c-1)];

%Dens_i=[1500 1500 1000.*ones(1,n-p_c-1-2) 1000.*ones(1,2*p_c-1)   1000.*ones(1,n-p_c-3) 1500 1500];

[BoomInfo] = Boom3DShape(BoomInfo,'Info','Density_variation',Dens_i,'Create_Stl');

BoomInfo.Mecc.I_rho
BoomInfo.Mecc.m
X_ini=[11.9000    1.1000   89.6000    3.9000    9.1000]*10;
X_ini=[ 11.8000    3.4000   73.7000   29.4000   10.2000]*10;


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
tic
[TOUT,YOUT_quat] = ode45(@(t,y)EquationOfMotionsQuaternion(t,y,BoomInfo,Tl_0),[0 tfin],Y0,options); %
toc
%[TOUT,YOUT_quat] = ode45(@(t,y)TestEquationOfMotionsQuaternion(t,y,BoomInfo,Tl_0),[0 tfin],Y0,options); %
%%
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
%%
Ecin=[];
E_pot=[];
Inerzia=BoomInfo.Mecc.I_rho;

% Iz=Inerzia(3,3);
% Ix= Inerzia(1,1);
% Iy=Inerzia(2,2);
% Ixy=Inerzia(1,2);
% Ixz=Inerzia(1,3);
% Iyz=Inerzia(2,3);
% 
% Inerzia=[Ix -Ixy -Ixz; -Ixy Iy -Iyz ; -Ixz -Iyz Iz ];
M=BoomInfo.Mecc.m;
for i=1:numel(TOUT)
    p=YOUT(i,4);
q=YOUT(i,5);
r=YOUT(i,6);
ux=YOUT(i,7);
uy=YOUT(i,8);
uz=YOUT(i,9);

Ecin(i)=1/2*([p q r]*Inerzia*[p;q;r]+[ux uy uz]*M*[ux;uy;uz]);
E_pot(i)=M*9.81*YOUT(i,12);
V(i)=norm([ux uy uz]);
Om(i)=norm([p q r]);
end
if max(diff(Ecin+E_pot))>0
    fprintf('Il boomerang sta creando energia IMPOSSIBILE \n');
end
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


linecolors={'r' 'y' 'c' 'g' 'b' 'k'};
[handles]=plotNy(TOUT(:),YOUT(:,6),1,...
    TOUT(:),YOUT(:,12),2,...
    TOUT(:),Ecin',3,...
    TOUT(:),E_pot',3,...
    TOUT(:),E_pot'+Ecin',3,...
    TOUT(:),V',4,...
    TOUT(:),Om',1,...
    'YAxisLabels',{ 'Angular Rate [rad/s]' 'Position [m]' 'Energy [J]' 'Velocità [m/s]'},...
    'Linewidth',1,...
    'XLim',[0,TOUT(end)],...
    'XAxisLabel','time[s]',...
    'TitleStr','Lancio Finale',...
    'FontSize',10,...
    'LegendString',{ 'r' 'z' 'E. Cinetica' 'E. Potenziale' 'E. Totale' 'Modulo V' 'Modulo Om'});
grid on
%% grafico
PlotTipDxSx(TOUT,YOUT,BoomInfo,Tl_0)
% PlotAeroForce(YOUT,TOUT,BoomInfo)
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
%%
dy=[];
dr_cont=[];
M=[];
for i=1:numel(TOUT)
I=BoomInfo.Mecc.I_rho;
m=BoomInfo.Mecc.m;

g=9.81;

Iz=I(3,3);
Ix= I(1,1);
Iy=I(2,2);
Ixy=I(1,2);
Ixz=I(1,3);
Iyz=I(2,3);

p=YOUT(i,5);
q=YOUT(i,6);
r=YOUT(i,7);
ux=YOUT(i,8);
uy=YOUT(i,9);
uz=YOUT(i,10);

[F,M]=AeroDynamics([ux;uy;uz],[p;q;r],BoomInfo);

M_pqr=[Ix -Ixy -Ixz; -Ixy Iy -Iyz ; -Ixz -Iyz Iz ];
M_pqr_inv=inv(M_pqr);
 R=[-(Iz-Iy)*q*r-(Ixy*p+Iyz*r)*r+(Ixz*p+Iyz*q)*q;...
     -(Ix-Iz)*p*r-(Iyz*q+Ixz*p)*p+(Ixy*p+Ixz*r)*r;...
     -(Iy-Ix)*p*q-(Ixz*r+Ixy*q)*q+(Iyz*r+Ixy*p)*p];

dy(:,i)=M_pqr\R;
dr_cont(:,i)=[M_pqr_inv(3,1)*R(1);M_pqr_inv(3,2)*R(2);M_pqr_inv(3,3)*R(3);[M_pqr_inv(3,1)*M(1);M_pqr_inv(3,2)*M(2);M_pqr_inv(3,3)*M(3)]];
Mi(:,i)=M;
end