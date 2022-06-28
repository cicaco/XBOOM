function [Cl,Cd,Cm] = AeroCoeff
T_0025=readmatrix('Cl_CD_naca0025.dat');
AoA=T_0025(:,2);
Cl=T_0025(:,3);
Cd=T_0025(:,4);
Cm=T_0025(:,5);

alpha_t=[-fliplr(AoA(2:end)')' ;AoA]*pi/180;
CD_t=[fliplr(Cd(2:end)')'; Cd];
CL_t=[-fliplr(Cl(2:end)')'; Cl];
CM_t=[-fliplr(Cm(2:end)')'; Cm];

% figure()
% plot(alpha_t,CL_t,'r');
% hold on
% plot(alpha_t,CD_t,'b');
% plot(alpha_t,CM_t,'k');
% grid on
% legend('Cl_0025','Cd_0025','Cm_0025','Cl_0012','Cd_0012');
% provo a costriure una function handle per creare funzioni veloci
Cd=@(AoA) 0;
Cl=@(AoA) 0;
Cm=@(AoA) 0;

for i=2:numel(CD_t)
    Cond=@(AoA) (AoA>=alpha_t(i-1)).*(AoA<alpha_t(i));
    Cd_i=@(AoA) Cond(AoA).*(CD_t(i-1)+(AoA-alpha_t(i-1))./(alpha_t(i)-alpha_t(i-1)).*(CD_t(i)-CD_t(i-1)));
    Cd = @(AoA) Cd_i(AoA)+Cd(AoA);
    Cl_i=@(AoA) Cond(AoA).*(CL_t(i-1)+(AoA-alpha_t(i-1))./(alpha_t(i)-alpha_t(i-1)).*(CL_t(i)-CL_t(i-1)));
    Cl = @(AoA) Cl_i(AoA)+Cl(AoA);
    Cm_i=@(AoA) Cond(AoA).*(CM_t(i-1)+(AoA-alpha_t(i-1))./(alpha_t(i)-alpha_t(i-1)).*(CM_t(i)-CM_t(i-1)));
    Cm = @(AoA) Cm_i(AoA)+Cm(AoA);
end
figure()
hold on
grid on
plot(alpha_t,Cd(alpha_t),'*b');
plot(alpha_t,Cl(alpha_t),'or');
plot(alpha_t,Cm(alpha_t),'+k');
end

