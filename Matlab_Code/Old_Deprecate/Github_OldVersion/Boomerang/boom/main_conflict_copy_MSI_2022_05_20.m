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
theta0=0*pi/180;
phi0=-60*pi/180;
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
ustart=Tl_0*[25;0;0];
fileID = fopen('debug11.txt','a+');
Y0=[theta0 phi0 psi0 0 0 10*2*pi  ustart(1) ustart(2) ustart(3) 0 0 0 ]';
[TOUT,YOUT] = ode45(@(t,y) LoreEquationOfMotions(t,y,fileID),[0 tfin],Y0);
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

%% prova
%geometria
R=0.30; %m
c=0.0488; %m
xac=0.0723; %va cambiato tra prima e seconda pala (DA FARE)

%sdr PALA j
sigma=120*pi/180;
coning=0; %tipo diedro (rot asse x2)
pitch=0; %pitch della pala (rot asse y3)

%matrice di rotazione da body a blade
Tj=[sin(sigma)*cos(pitch)+cos(sigma)*sin(coning)*sin(pitch), -cos(sigma)*cos(pitch)+sin(sigma)*sin(coning)*sin(pitch), -cos(coning)*sin(pitch)
    cos(sigma)*cos(coning), sin(sigma)*cos(coning), +sin(coning)
    sin(sigma)*sin(pitch)-cos(sigma)*sin(coning)*cos(pitch), -cos(sigma)*sin(pitch)-sin(sigma)*sin(coning)*cos(pitch), cos(coning)*cos(pitch)];

x_tip_bodyframe=[xac;0;0]+Tj'*[0;R;0];
x_tipsx=[];
%% RAF:ho aggiunto questo
load 2DPlant.mat

points = [geo.LEL; geo.TEL; geo.LER; geo.TER];

%these are points in body frame
%hp baricenter in zero -> clearly wrong
points_3D = [points, zeros(length(points),1)]*10;
%% 

%this is done to exchange the y axis
points_3D(:,2) = points_3D(:,2);
points_3D(:,1) = -points_3D(:,1)+0.0723; 
figure;

for i=1:length(TOUT)
theta=(YOUT(i,1));
    phi=(YOUT(i,2));
    psi=(YOUT(i,3));
    T0=[cos(theta)*cos(psi), cos(theta)*sin(psi), -sin(theta)
    -cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi), cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi), sin(phi)*cos(theta)
    sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi), -sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi), cos(phi)*cos(theta)];
    x_trans = [YOUT(i,10); YOUT(i,11); YOUT(i,12)]+T0'*[x_tip_bodyframe];
    
    x_rot=[];
%     for punti=1:400
    x_rot  = [YOUT(i,10); YOUT(i,11); YOUT(i,12)]' + points_3D*T0;
%     end
    
    plot3(x_rot(:,1), x_rot(:,2), x_rot(:,3), 'r');
    xlim([x_trans(1)-0.7,x_trans(1)+0.7]);
    ylim([x_trans(2)-0.7,x_trans(2)+0.7]);
    zlim([x_trans(3)-0.7,x_trans(3)+0.7]);
    hold off;
    grid on;
    M(i) = getframe;
x_tipsx=[x_tipsx  [YOUT(i,10); YOUT(i,11); YOUT(i,12)]+T0'*[x_tip_bodyframe]];
end

%%
plot3(YOUT(:,10),YOUT(:,11),YOUT(:,12))
hold on
plot3(x_tipsx(1,:),x_tipsx(2,:),x_tipsx(3,:),'g')

xlabel('X');
ylabel('Y');
zlabel('Z');
figure;
movie(M)
grid on;
