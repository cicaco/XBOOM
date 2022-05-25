clear all
close all
addpath(genpath('3DSolidGeneration'));
addpath(genpath('Mecc'));
addpath(genpath('General'));
addpath(genpath('Aero'));
%% DATA 
% Profile 2D Shape
Profile2D=importdata('Naca0012.dat');
Profile2D_2=importdata('Naca0012.dat'); 
%Essenziale mettere due volte lo zero!!
X_0040=[fliplr(Profile2D_2.data(1:66,1));0;(Profile2D_2.data(67:end,1))];
Z_0040=[fliplr(Profile2D_2.data(1:66,2));0;(Profile2D_2.data(67:end,2))];
%Sistemo il plot in senso orario partendo da 0,0
Xp_2d_b=[Profile2D.data(2:67,1) ; fliplr(Profile2D.data(68:end,1)')'];
Zp_2d_b=[Profile2D.data(2:67,2) ; fliplr(Profile2D.data(68:end,2)')'];
% Xp_2d_b=X_0040;
%   Zp_2d_b=Z_0040;
Chord=0.05;
%Forma 3D
x_0=0; %Corda dimensione
y_0=0;
z_0=0;
x_f=0.05;
y_f=0;
z_f=0;
k_1=0.05; % origin point tangent ,y
k_2= 0.1; %mid point, trailing edge x
k_3= 0.15; %mid point  y
k_4=0.15; %Tip point tangent sx, x
k_5=0.3; %Tip point  y
k_6=0.2; %Tip point  x
k_7=0.25; %Tip point tangent dx, x
k_8=0.2; %mid point leading edge, x


Perc_center=30; %da che punto iniziare a flippare il profilo
Perc_middle=40; % Percentualeoltre la quale inizia la parte finale del profilo
Filename='DAJE.stl';

%Generation of the Structure
BoomData.Profil2D.X=Xp_2d_b;
BoomData.Profil2D.Z=Zp_2d_b;
BoomData.Profil2D.Chord=Chord;
BoomData.Profil3D.x0=x_0;
BoomData.Profil3D.y0=y_0;
BoomData.Profil3D.z0=x_0;
BoomData.Profil3D.xf=x_f;
BoomData.Profil3D.yf=y_f;
BoomData.Profil3D.zf=z_f;
BoomData.Profil3D.k1=k_1;
BoomData.Profil3D.k2=k_2;
BoomData.Profil3D.k3=k_3;
BoomData.Profil3D.k4=k_4;
BoomData.Profil3D.k5=k_5;
BoomData.Profil3D.k6=k_6;
BoomData.Profil3D.k7=k_7;
BoomData.Profil3D.k8=k_8;
BoomData.Profil3D.Perc_center=Perc_center;
BoomData.Profil3D.Perc_middle=Perc_middle;
BoomData.filename=Filename;

[S,RBP,Plant3D]=Boom3DShape(BoomData); % VELOCIZZARE IL CODICE ASSOLUTAMENTE
%%
G=[RBP.centroid(1) 0 0];
close all
figure()
plot(Plant3D(1,:),Plant3D(2,:),'*b');
plot(G(1),G(2),'*r');

hold on
Chord=Plant3D(1,1:100)+(fliplr(Plant3D(1,101:200))-(Plant3D(1,1:100)))./4;
Aerc=[fliplr(Chord) Chord;fliplr(Plant3D(2,1:100)) -Plant3D(2,1:100)];
plot(Aerc(1,:),Aerc(2,:),'*g');
axis equal
figure()
title('Boomerang frame');
plot3(G(1),G(2),G(3),'*r');
hold on
plot3(S(:,1),S(:,2),S(:,3),'*b');

grid on
axis equal
% Devo cambiare sistema di riferimento e portarlo al baricentro
% translazione di X e rotazione sull'asse Z di 180° come nel paper 
R=[-1 0; 0 1];
Trasl=G(1:2)';
G_tras=G(1:2)'-Trasl;
Aerc_tras=R*(Aerc-Trasl.*ones(2,200));

Geom.Aerc_tras=Aerc_tras;
Geom.Plant3D=Plant3D;
close all
figure()
title('Cambio di sistema di riferimento');
plot3(Aerc_tras(1,:),Aerc_tras(2,:),zeros(size(Aerc_tras(2,:))),'*g');
hold on
plot3(G_tras(1),G_tras(2),0,'*r');
grid on
axis equal
P_aerc=[Aerc_tras(1,:)' Aerc_tras(2,:)'   zeros(size(Aerc_tras(2,:)))'];


%%


Rho=1500;
I=Rho*RBP.inertia_tensor;
m=RBP.volume*Rho
theta0=0*pi/180;
phi0=40*pi/180;
psi0=0*pi/180;
% Tl_0_prova=[cos(psi)*cos(theta) cos(theta)*sin(psi) sin(theta);...
% -cos(phi)*sin(psi)-cos(psi)*sin(phi)*sin(theta)   cos(phi)*cos(psi)-sin(phi)*sin(psi)*sin(theta) cos(theta)*sin(phi);...
% sin(phi)*sin(psi)-cos(phi)*cos(psi)*sin(theta) -cos(psi)*sin(phi)-cos(phi)*sin(psi)*sin(theta) cos(phi)*cos(theta)];

Tl_0=[cos(theta0)*cos(psi0), cos(theta0)*sin(psi0), -sin(theta0)
    -cos(phi0)*sin(psi0)+sin(phi0)*sin(theta0)*cos(psi0), cos(phi0)*cos(psi0)+sin(phi0)*sin(theta0)*sin(psi0), sin(phi0)*cos(theta0)
    sin(phi0)*sin(psi0)+cos(phi0)*sin(theta0)*cos(psi0), -sin(phi0)*cos(psi0)+cos(phi0)*sin(theta0)*sin(psi0), cos(phi0)*cos(theta0)];

% theta=0;
% phi=0*pi/180;

% T0=[cos(theta)*cos(psi), -cos(theta)*sin(psi), -sin(theta)
%     cos(phi)*sin(psi)-sin(phi)*sin(theta)*cos(psi), cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi), -sin(phi)*cos(theta)
%     sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi), +sin(phi)*cos(psi)-cos(phi)*sin(theta)*sin(psi), cos(phi)*cos(theta)];
tfin=3;
ustart=Tl_0*[20;0;4];
fileID = fopen('debug.txt','a+');
options = odeset('RelTol',1e-4,'AbsTol',1e-6);
Y0=[theta0 phi0 psi0 0  0 10*2*pi ustart(1) ustart(2) ustart(3) 0 0 1.5 ]';
[TOUT,YOUT] = ode45(@(t,y) LoreEquationOfMotions(t,y,fileID,I,m,Geom),[0 tfin],Y0,options);
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
    'XLim',[0,tfin],...
    'XAxisLabel','time[s]',...
    'TitleStr','Lancio Finale',...
    'FontSize',10,...
    'LegendString',{'Theta' 'Phi' 'Psi' 'r' 'p' 'q' 'x' 'y' 'z'});
grid on
%%
figure()
plot3(YOUT(:,10),YOUT(:,11),YOUT(:,12))
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;

