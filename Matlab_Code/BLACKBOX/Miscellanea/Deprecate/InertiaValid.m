%% Script per creare la Geometria 3D
clear all 
close all
addpath(genpath('BLACKBOX'));

%% Input Data
Chord=0.045;


l=0.3; % lunghezza della pala avente un profilo 2D definito, NON corrisponde alla lunghezza del boomerang
delta=30*pi/180; %Angolo di freccia
beta=10*pi/180; %Angolo di Diedro
pitch=10*pi/180; %Pitch angle
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
plot(Xp,Zp,'r','linewidth',1.2);
hold on
plot(Xp_flip,Zp_flip,'b','linewidth',1.2);
axis equal

[n,~]=size(Xp);
%Clock-wise direction regeneration
Xp_flip=[fliplr(Xp_flip(1:n/2)')';fliplr(Xp_flip(n/2+1:end)')'];
Zp_flip=[fliplr(Zp_flip(1:n/2)')';fliplr(Zp_flip(n/2+1:end)')'];
[X_trans,Z_trans] = Profile2d_Trans(Xp,Zp,Zp_flip,5);
figure()
h1=plot(Xp,Zp,'r','linewidth',1.2);
hold on
h2=plot(Xp_flip,Zp_flip,'b','linewidth',1.2);
axis equal
for i=2:5-1
    h3=plot(X_trans(i,:),Z_trans(i,:),'k');
end
grid on 
xlabel('X','fontsize',11,'interpreter','latex');
set(gca,'TickLabelInterpreter','latex')
ylabel('Z','fontsize',11,'interpreter','latex');
legend([h1 h2 h3],'Profilo Sinistro','Profilo Destro','Profili di Transizioni');
title('Profili di Transizione','fontsize',12,'interpreter','latex');
%% Creazione dell Info Box
BoomInfo.Pianta.l=l;
BoomInfo.Pianta.freccia=delta;
BoomInfo.Pianta.diedro=beta;
BoomInfo.Pianta.pitch=pitch;

BoomInfo.Geom3D.PARA=PARA;
BoomInfo.Profile.Chord=Chord;
BoomInfo.Profile.Xp_dx=Xp;
BoomInfo.Profile.Xp_sx=Xp_flip;
BoomInfo.Profile.Zp_dx=Zp;
BoomInfo.Profile.Zp_sx=Zp_flip;

%% Geometry
num=[10 50 100 500 1000 5000];
p_c=[10 50 100 500 1000 5000];
for i=1:6
n=num(i)+p_c(i);
i
BoomInfo.Geom3D.p_c=p_c(i);
BoomInfo.Geom3D.num=num(i);
Dens_i=[1100.*ones(1,n-p_c(i)-1) 900*ones(1,2*p_c(i)-1) 1100.*ones(1,n-p_c(i)-1)];
R=1000;
Dens_i=[R.*ones(1,n-p_c(i)-1) R*ones(1,2*p_c(i)-1) R.*ones(1,n-p_c(i)-1)];

%Dens_i=[1500 1500 1000.*ones(1,n-p_c-1-2) 1000.*ones(1,2*p_c-1)   1000.*ones(1,n-p_c-3) 1500 1500];

[BoomInfo] = Boom3DShape(BoomInfo,'Info','Density_variation',Dens_i);

I(:,:,i)=BoomInfo.Mecc.I_rho;
m(i)=BoomInfo.Mecc.m;
%rho*BoomInfo.Mecc.I
%BoomInfo.Mecc.I_rho=rho*BoomInfo.Mecc.I;
%BoomInfo.Mecc.m=BoomInfo.Mecc.V*1000;
end
P_tot=BoomInfo.Geom3D.Profile;
save('P3D.mat','P_tot');

%%
n=num+p_c;
