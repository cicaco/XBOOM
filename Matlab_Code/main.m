%%  MAIN SCRIPT 
% all the function are recalled here
clear all;
close all;
clc;

addpath(genpath('Aero'));
addpath(genpath('Mecc'));
%run Main_geometry.m

Y0=[0 2*pi 10*2*pi 80*pi/180 0 0 26*cos(4*pi/180) 0 26*sin(4*pi/180) 0 0 0 ]';
[TOUT,YOUT] = ode45(@EquationOfMotions,[0 3],Y0);
%%
plot3(YOUT(:,10),YOUT(:,11),YOUT(:,12))
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;