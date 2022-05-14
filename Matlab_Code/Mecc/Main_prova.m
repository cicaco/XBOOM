clear
clc
close all


Y0=[0 2*pi 10*2*pi 80*pi/180 0 0 10*cos(4*pi/180) 0 10*sin(4*pi/180) 0 0 0 ]';
[TOUT,YOUT] = ode45(@EquationOfMotions,[0 3],Y0);
%%
plot3(YOUT(:,10),YOUT(:,11),YOUT(:,12))
xlabel('X');
ylabel('Y');
zlabel('Z');