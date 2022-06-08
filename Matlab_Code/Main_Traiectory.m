%%  MAIN SCRIPT 
addpath(genpath('BLACKBOX'));

% all the function are recalled here
clear all;
close all;
clc;
x=linspace(-pi,pi,100);
y=zeros(100,1);
for i=1:100
y(i)=CD_new(x(i));
end
figure()
plot(x,y)
%%
theta0=0*pi/180;
phi0=-70*pi/180;  % initial inclination of the boomerang -70 / -90 degrees; baseline -85 degrees
psi0=0*pi/180;
r0= 100*2*pi; % initial condition on spin rate 12/15 Hz; baseline 13
z0= 1; % initial altitude
global m;
m=10;

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


tfin=20;
ustart=Tl_0*[25;0;0];
fileID = fopen('debug.txt','a+');
options = odeset('Events', @Events,'RelTol',1e-4,'AbsTol',1e-6);
Y0=[theta0 phi0 psi0 0 0 r0  ustart(1) ustart(2) ustart(3) 0 0 z0 ]';
[TOUT,YOUT] = ode45(@(t,y) EquationOfMotions(t,y,fileID),[0 tfin],Y0,options);
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
R=0.30; %m
c=0.0488; %m
xac=0.0723; %va cambiato tra prima e seconda pala (DA FARE)

%sdr PALA j
sigma=120*pi/180;
coning=0; %tipo diedro (rot asse x2)
pitch=5*pi/180; %pitch della pala (rot asse y3)

%matrice di rotazione da body a blade
Tj=[sin(sigma)*cos(pitch)+cos(sigma)*sin(coning)*sin(pitch), -cos(sigma)*cos(pitch)+sin(sigma)*sin(coning)*sin(pitch), -cos(coning)*sin(pitch)
    cos(sigma)*cos(coning), sin(sigma)*cos(coning), +sin(coning)
    sin(sigma)*sin(pitch)-cos(sigma)*sin(coning)*cos(pitch), -cos(sigma)*sin(pitch)-sin(sigma)*sin(coning)*cos(pitch), cos(coning)*cos(pitch)];

x_tipsx_bodyframe=[xac;0;0]+Tj'*[0;R;0];

%sdr PALA dx
sigma=240*pi/180;
coning=0; %tipo diedro (rot asse x2)
pitch=5*pi/180; %pitch della pala (rot asse y3)
%matrice di rotazione da body a blade
Tj=[sin(sigma)*cos(pitch)+cos(sigma)*sin(coning)*sin(pitch), -cos(sigma)*cos(pitch)+sin(sigma)*sin(coning)*sin(pitch), -cos(coning)*sin(pitch)
    cos(sigma)*cos(coning), sin(sigma)*cos(coning), +sin(coning)
    sin(sigma)*sin(pitch)-cos(sigma)*sin(coning)*cos(pitch), -cos(sigma)*sin(pitch)-sin(sigma)*sin(coning)*cos(pitch), cos(coning)*cos(pitch)];
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

%%
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

%% Aerodynamic
R=@(theta,phi,psi) [cos(theta)*cos(psi), cos(theta)*sin(psi), -sin(theta)
    -cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi), cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi), sin(phi)*cos(theta)
    sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi), -sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi), cos(phi)*cos(theta)];

ux=YOUT(:,7);
uy=YOUT(:,8);
uz=YOUT(:,9);
p=YOUT(:,4);
q=YOUT(:,5);
r=YOUT(:,6);
theta=YOUT(:,1);
phi=YOUT(:,2);
psi=YOUT(:,3);
num=numel(p);
F=zeros(3,num);
M=zeros(3,num);
for i=1:num
[F(:,i),M(:,i)]=AeroDynamics([ux(i);uy(i);uz(i)],[p(i);q(i);r(i)]);
F(:,i)=R(theta(i),phi(i),psi(i))'*F(:,i);
M(:,i)=R(theta(i),phi(i),psi(i))'*M(:,i);
end

figure()
subplot(3,2,1)
plot(TOUT(:),F(1,:)','r','linewidth',1);
legend('Fx');
grid on

subplot(3,2,2)
plot(TOUT(:),M(1,:)','r','linewidth',1);
legend('Mx');
grid on

subplot(3,2,3)
plot(TOUT(:),F(2,:)','b','linewidth',1);
legend('Fy');
grid on

subplot(3,2,4)
plot(TOUT(:),M(2,:)','b','linewidth',1);
legend('My');
grid on

subplot(3,2,5)
plot(TOUT(:),F(3,:)','g','linewidth',1);
legend('Fz');
grid on

subplot(3,2,6)
plot(TOUT(:),M(3,:)','g','linewidth',1);
legend('Mz');
grid on

sgtitle('Moment and force in inertial frame');
%% creazione traiettoria per blender
Time=TOUT(:);
x=YOUT(:,10);
y=YOUT(:,11);
z=YOUT(:,12);
Phi=YOUT(:,2)*180/pi;
Psi=YOUT(:,3)*180/pi;
Theta=YOUT(:,1)*180/pi;
save('Traiectory.mat','Time','Theta' ,'Phi' ,'Psi' , 'x' ,'y' ,'z');