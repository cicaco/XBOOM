clear all
close all
T_0025=readmatrix('Cl_CD_naca0025.dat');
AoA=T_0025(:,2);
Cl=T_0025(:,3);
Cd=T_0025(:,4);
Cm=T_0025(:,5);

AoA_tot=[-fliplr(AoA')' ;AoA];
Cd_tot=[fliplr(Cd')'; Cd];
Cl_tot=[-fliplr(Cl')'; Cl];
Cm_tot=[-fliplr(Cm')'; Cm];

figure()
plot(AoA_tot,Cl_tot,'r');
hold on
plot(AoA_tot,Cd_tot,'b');
plot(AoA_tot,Cm_tot,'k');

grid on
T_0012=readmatrix('Cl_CD_naca0012_re4_4.dat');
AoA=T_0012(:,2);
Cl=T_0012(:,3);
Cd=T_0012(:,4);
AoA_tot=[-fliplr(AoA')' ;AoA];
Cd_tot=[fliplr(Cd')'; Cd];
Cl_tot=[-fliplr(Cl')'; Cl];
plot(AoA_tot,Cl_tot,'g');
hold on
plot(AoA_tot,Cd_tot,'c');
grid on
legend('Cl_0025','Cd_0025','Cm_0025','Cl_0012','Cd_0012');

%Creazione della function handle a tratti