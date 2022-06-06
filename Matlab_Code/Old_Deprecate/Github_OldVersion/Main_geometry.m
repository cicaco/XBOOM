clear all
close all
addpath(genpath('3DSolidGeneration'));
%% DATA 
% Profile 2D Shape
Profile2D=importdata('Naca0012.dat');
%Essenziale mettere due volte lo zero!!


%Sistemo il plot in senso orario partendo da 0,0
Xp_2d_b=[Profile2D.data(2:67,1) ; fliplr(Profile2D.data(68:end,1)')'];
Zp_2d_b=[Profile2D.data(2:67,2) ; fliplr(Profile2D.data(68:end,2)')'];


%% Parametri 3D
close all
D = 30*pi/180;
Beta = 10*pi/180;
l=0.3;
Chord= 0.05;
Perc_center=5;
Perc_tip=95;
r=1;

% 1 step creo il centro aerodinamico
% creo come sempre mezza pala
y_fin=l*cos(D);
x=@(y)-y.*tan(D);
% z=@(y) sin(Beta).*y.*(1+tan(D)^2);
num=100;
y_l=linspace(0,y_fin,num);
x_l=x(y_l);
% z_left=z(y_left);
figure()


plot(x_l,y_l,'*r');
set(gca,'Ydir','reverse')
set(gca,'Xdir','reverse')
grid on
hold on
axis equal

% ora devo cercare di far il raccordo centrale 


Y_t=y_l(Perc_center);
X_t=x(Y_t);
plot(x(Y_t),Y_t,'*g');

Dx=2*Y_t;
Gamma=atan(abs(Y_t/Dx));
r=sqrt(Y_t^2+Dx^2);
G_angle=linspace(0,Perc_center-2,Perc_center-1).*Gamma/4;
y_c=[sin(G_angle)].*r;
x_c=-(-X_t+Dx).*ones(1,4)+[cos(G_angle)].*r;
y_le=y_l(Perc_center:end);
x_le=x_l(Perc_center:end);

y_left=[ y_c y_le];
x_left=[ x_c x_le];
z_left=sin(Beta).*sqrt(y_left.^2+x_left.^2);


%rotazione di beta di x ed y
%%
figure()
plot3(x_left,y_left,z_left,'*g');
axis equal
set(gca,'Ydir','reverse')
grid on
set(gca,'Xdir','reverse')

%Rotazione dato asse