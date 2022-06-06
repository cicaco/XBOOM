clear all;
close all;
clc

a       = linspace(-10,10, 21);
Re      = 50000; %boomerang should have this Re (almost)
Mach    = 0;
airfoil = 'NACA0004'

[pol,foil] = xfoil(airfoil,a,Re,Mach,'panels n 330', 'oper iter 1000')

plot(pol.alpha, pol.CL, 'o', 'LineWidth', 2)
title('CL_alpha')
grid on;

figure;
plot(pol.CD, pol.CL, '-.', 'LineWidth', 2)
title('Polare')
grid on;

for ii = 12:20
    figure
    plot(foil.xcp, -foil.cp(:,ii));
    grid on;
end
