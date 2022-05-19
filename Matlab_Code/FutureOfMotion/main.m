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

% theta=0;
% phi=0*pi/180;

% T0=[cos(theta)*cos(psi), -cos(theta)*sin(psi), -sin(theta)
%     cos(phi)*sin(psi)-sin(phi)*sin(theta)*cos(psi), cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi), -sin(phi)*cos(theta)
%     sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi), +sin(phi)*cos(psi)-cos(phi)*sin(theta)*sin(psi), cos(phi)*cos(theta)];
tfin=5;
ustart=Tl_0*[15;0;2];
fileID = fopen('debug11.txt','a+');
Y0=[theta0 phi0 psi0 0 0 20*2*pi  ustart(1) ustart(2) ustart(3) 0 0 0 ]';
[TOUT,YOUT] = ode23(@(t,y) LoreEquationOfMotions(t,y,fileID),[0 tfin],Y0);
fclose(fileID);
%%
plot3(YOUT(:,10),YOUT(:,11),YOUT(:,12))
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;
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

