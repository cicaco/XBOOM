clear all
close all
addpath(genpath('3DSolidGeneration'));
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
k_4=0.2; %Tip point tangent sx, x
k_5=0.3; %Tip point  y
k_6=0.3; %Tip point  x
k_7=0.3; %Tip point tangent dx, x
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

[Shape,RBP]=Boom3DShape(BoomData);
Rho=1500;
I=Rho*RBP.inertia_tensor;
%%
addpath(genpath('Mecc'));

