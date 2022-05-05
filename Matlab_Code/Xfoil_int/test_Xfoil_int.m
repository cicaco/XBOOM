clear all;
close all;
clc

a       = linspace(0,10, 11);
Re      = 50000; %boomerang should have this Re (almost)
Mach    = 0;
airfoil = 'NACA0012'

[pol,foil] = xfoil(airfoil,a,Re,Mach,'panels n 330', 'oper iter 50')

