%%  MAIN SCRIPT 
% all the function are recalled here
clear all;
close all;
clc;
x=linspace(-pi,pi,100);
y=zeros(100,1);
for i=1:100
y(i)=CD(x(i));
end
figure()
plot(x,y)
addpath(genpath('Aero'));

%%
theta0=0;
phi0=70*pi/180;
psi0=0*pi/180;
% Tl_0_prova=[cos(psi)*cos(theta) cos(theta)*sin(psi) sin(theta);...
% -cos(phi)*sin(psi)-cos(psi)*sin(phi)*sin(theta)   cos(phi)*cos(psi)-sin(phi)*sin(psi)*sin(theta) cos(theta)*sin(phi);...
% sin(phi)*sin(psi)-cos(phi)*cos(psi)*sin(theta) -cos(psi)*sin(phi)-cos(phi)*sin(psi)*sin(theta) cos(phi)*cos(theta)];

Tl_0=[cos(theta0)*cos(psi0), cos(theta0)*sin(psi0), -sin(theta0)
    -cos(phi0)*sin(psi0)+sin(phi0)*sin(theta0)*cos(psi0), cos(phi0)*cos(psi0)+sin(phi0)*sin(theta0)*sin(psi0), sin(phi0)*cos(theta0)
    sin(phi0)*sin(psi0)+cos(phi0)*sin(theta0)*cos(psi0), -sin(phi0)*cos(psi0)+cos(phi0)*sin(theta0)*sin(psi0), cos(phi0)*cos(theta0)];
R=@(theta,phi,psi) [cos(theta)*cos(psi), cos(theta)*sin(psi), -sin(theta)
    -cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi), cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi), sin(phi)*cos(theta)
    sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi), -sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi), cos(phi)*cos(theta)];

% theta=0;
% phi=0*pi/180;

% T0=[cos(theta)*cos(psi), -cos(theta)*sin(psi), -sin(theta)
%     cos(phi)*sin(psi)-sin(phi)*sin(theta)*cos(psi), cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi), -sin(phi)*cos(theta)
%     sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi), +sin(phi)*cos(psi)-cos(phi)*sin(theta)*sin(psi), cos(phi)*cos(theta)];
tfin=5;
ustart=Tl_0*[15;0;2];
fileID = fopen('debug11.txt','a+');
Y0=[theta0 phi0 psi0 0 0 20*2*pi  ustart(1) ustart(2) ustart(3) 0 0 0]';
[TOUT,YOUT] = ode23(@(t,y) LoreEquationOfMotions(t,y,fileID),[0 tfin],Y0);
fclose(fileID);
%%
plot3(YOUT(:,10),YOUT(:,11),YOUT(:,12))
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;
%%
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
[F(:,i),M(:,i)]=aero([ux(i);uy(i);uz(i)],[p(i);q(i);r(i)]);
F(:,i)=R(theta(i),phi(i),psi(i))'*F(:,i);
M(:,i)=R(theta(i),phi(i),psi(i))'*M(:,i);
end
linecolors={'r' 'y' 'c' 'g' 'b' 'k'};
%%
figure()
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
%%
figure()
[handles]=plotNy(TOUT(:),F(1,:)',1,...
TOUT(:),F(2,:)',1,...
TOUT(:),F(3,:)',1,...
    TOUT(:),M(1,:)',2,...
    TOUT(:),M(2,:)',2,...
    TOUT(:),M(3,:)',2,...
    'YAxisLabels',{ 'Force [N]' 'Momentum[Nm]' },...
    'Linewidth',1,...
    'XLim',[0,tfin],...
    'XAxisLabel','time[s]',...
    'TitleStr','Lancio Finale',...
    'FontSize',10,...
    'LegendString',{'Fx' 'Fy' 'Fz' 'Mx' 'My' 'Mz'});
grid on