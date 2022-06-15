%Initial Condition
clear all
close all
addpath(genpath('BLACKBOX'));

load('Boom.mat');
num=BoomInfo.Geom3D.num;
p_c=BoomInfo.Geom3D.p_c;
l=BoomInfo.Pianta.l;
theta0=10*pi/180;
phi0=45*pi/180;  % initial inclination of the boomerang -70 / -90 degrees; baseline -85 degrees
psi0=180*pi/180;
Tl_0=[cos(theta0)*cos(psi0), cos(theta0)*sin(psi0), -sin(theta0)
    -cos(phi0)*sin(psi0)+sin(phi0)*sin(theta0)*cos(psi0), cos(phi0)*cos(psi0)+sin(phi0)*sin(theta0)*sin(psi0), sin(phi0)*cos(theta0)
    sin(phi0)*sin(psi0)+cos(phi0)*sin(theta0)*cos(psi0), -sin(phi0)*cos(psi0)+cos(phi0)*sin(theta0)*sin(psi0), cos(phi0)*cos(theta0)];
% BoomInfo.Mecc.I_rho = 3*BoomInfo.Mecc.I_rho;

% BoomInfo.Mecc.I_rho =10^-4*[ 14 8 0; 8 14 0; 0 0 27];
% BoomInfo.Mecc.m=0.075;
%%
global alpha_t 
global CM_t
global CD_t
global CL_t

T_0025=readmatrix('Cl_CD_naca0025.dat');
AoA=T_0025(:,2);
Cl=T_0025(:,3);
Cd=T_0025(:,4);
Cm=T_0025(:,5);

alpha_t=[-fliplr(AoA(2:end)')' ;AoA];
CD_t=[fliplr(Cd(2:end)')'; Cd];
CL_t=[-fliplr(Cl(2:end)')'; Cl];
CM_t=[-fliplr(Cm(2:end)')'; Cm];

figure(2)
plot(alpha_t,CL_t,'r');
hold on
plot(alpha_t,CD_t,'b');
plot(alpha_t,CM_t,'k');

legend('Cl_0025','Cd_0025','Cm_0025','Cl_0012','Cd_0012');
%%
tfin=4;
U=(Tl_0*[15;0;0])';
Om=[ 0 0 13*2*pi];
P_tot=BoomInfo.Geom3D.Profile;
figure(1)
for i=1:2*(num+p_c)-2
    P_ruot=Tl_0'*P_tot(3*i-2:3*i,:);
    plot3(P_ruot(1,:),P_ruot(2,:),P_ruot(3,:),'k');
    hold on
    axis equal
    grid on
end
h1=quiver3(0.0,0,0,1,0 ,0,'b','linewidth',1);
quiver3(0.0,0,0,0,1 ,0,'b','linewidth',1);
quiver3(0.0,0,0,0,0 ,1,'b','linewidth',1);
plot3([0 -1],[0 0], [0 0],'--b','linewidth',1);
plot3([0 0],[0 -1], [0 0],'--b','linewidth',1);
plot3([0 0],[0 0], [0 -1],'--b','linewidth',1);
legend([h1],{'Asse-Terreno'});
xlabel('X');
ylabel('Y');
zlabel('Z');
xlim([-2*l,2*l]);
%% Data Import
Cd=BoomInfo.Aero.Cd;
Cl=BoomInfo.Aero.Cl;
Cm=BoomInfo.Aero.Cm;
I_origin_Sx=BoomInfo.Aero.P_origin_Sx;
I_origin_Dx=BoomInfo.Aero.P_origin_Dx;
Chord=BoomInfo.Profile.Chord; %m
num=BoomInfo.Geom3D.num;
p_c=BoomInfo.Geom3D.p_c;
delta=BoomInfo.Pianta.freccia;
beta=BoomInfo.Pianta.diedro; %(rot asse x2)
pitch=BoomInfo.Pianta.pitch; 
C_fin_rot=BoomInfo.Geom3D.C_aer;
CG=BoomInfo.Mecc.CG;
% Calcolo velocità in questi punti
P_dx=C_fin_rot(:,1:num)';
P_sx=C_fin_rot(:,num+2*p_c:end)';

V_dx=zeros(size(P_dx,1),3);
V_sx=zeros(size(P_dx,1),3);
rho=1.225;
%matrice di rotazione da body a blade
R_Aer_Body=@(Fr,Di) [cos(Fr) -sin(Fr) 0; sin(Fr) cos(Fr) 0; 0 0 1]...
    *[1 0 0; 0 cos(Di) -sin(Di); 0 sin(Di) cos(Di)];
R_Aer_Body_all=@(Fr,Di,AoA,Pitch) [cos(Fr) -sin(Fr) 0; sin(Fr) cos(Fr) 0; 0 0 1]...
    *[1 0 0; 0 cos(Di) -sin(Di); 0 sin(Di) cos(Di)]*...
    [cos(AoA+Pitch) 0 +sin(AoA+Pitch); 0 1 0; -sin(AoA+Pitch) 0 cos(AoA+Pitch)];
D_dir_sx=R_Aer_Body_all(-delta,-beta,0,-pitch)*[1;0;0];
L_dir_sx=R_Aer_Body_all(-delta,-beta,0,-pitch)*[0;0;1];
Par_sx=R_Aer_Body(delta,beta)*[1;0;0];
Perp_sx=R_Aer_Body(delta,beta)*[0;0;1];
V_aer_sx=zeros(size(P_dx,1),3);
AoA_sx=zeros(size(P_dx,1),1);
L_aer_i=zeros(size(P_dx,1),3);
AoA_Geom=zeros(size(P_dx,1),1);
l_i_sx=vecnorm(I_origin_Sx-P_sx');
l_i_dx=vecnorm(I_origin_Dx-P_dx');

for i=1:size(P_sx,1)
    
    V_dx(i,:)=U'+cross(Om',P_dx(i,:))';
    V_sx(i,:)=U'+cross(Om',P_sx(i,:))';
    %text(P_sx(i,1),P_sx(i,2),P_sx(i,3),string(i));

    % Devo considerare la forza aerodinamica nel sistema di riferimento
    % aerodinamico
    V_aer_sx(i,:)=-R_Aer_Body(delta,beta)*V_sx(i,:)';
    AoA_sx(i)=atan2(-V_aer_sx(i,3),-V_aer_sx(i,1))*180/pi;
    AoA_Geom_sx(i)=wrapTo2Pi(AoA_sx(i)*pi/180)*180/pi+pitch*180/pi;
    AoA_Geom_sx(i)=AoA_Geom_sx(i)-360*(AoA_Geom_sx(i)>=180);
    L_aer_sx=1/2*rho*norm(V_aer_sx(i,[1 3]))^2*Chord*[0;0;Cl(AoA_Geom_sx(i))];
    D_aer_sx=1/2*rho*norm(V_aer_sx(i,[1 3]))^2*Chord*[Cd(AoA_Geom_sx(i));0;0];
    M_aer_sx=1/2*rho*norm(V_aer_sx(i,[1 3]))^2*Chord^2*[0;Cm(AoA_Geom_sx(i));0];
    F_body_sx(i,:)=R_Aer_Body_all(delta,beta,AoA_sx(i)*pi/180,0)*(L_aer_sx+D_aer_sx);
    M_body_sx(i,:)=R_Aer_Body_all(delta,beta,AoA_sx(i)*pi/180,0)*(M_aer_sx);
% Pala dx
    V_aer_dx(i,:)=-R_Aer_Body(-delta,-beta)*V_dx(i,:)';
    AoA_dx(i)=atan2(V_aer_dx(i,3),-V_aer_dx(i,1))*180/pi;
    AoA_Geom_dx(i)=wrapTo2Pi(AoA_dx(i)*pi/180)*180/pi+pitch*180/pi;
    AoA_Geom_dx(i)=AoA_Geom_dx(i)-360*(AoA_Geom_dx(i)>=180);
    L_aer_dx=1/2*rho*norm(V_aer_dx(i,[1 3]))^2*Chord*[0;0;Cl(AoA_Geom_dx(i))];
    D_aer_dx=1/2*rho*norm(V_aer_dx(i,[1 3]))^2*Chord*[Cd(AoA_Geom_dx(i));0;0];
    M_aer_dx=1/2*rho*norm(V_aer_dx(i,[1 3]))^2*Chord^2*[0;Cm(AoA_Geom_dx(i));0];
    F_body_dx(i,:)=R_Aer_Body_all(-delta,-beta,AoA_dx(i)*pi/180,0)*(L_aer_dx+D_aer_dx);
    M_body_dx(i,:)=R_Aer_Body_all(-delta,-beta,AoA_dx(i)*pi/180,0)*(M_aer_dx);
end

F_sx=sum((F_body_sx(1:num-1,:)+F_body_sx(2:num,:)).*(l_i_sx(2:num)-l_i_sx(1:num-1))'./2,1);
F_dx=sum((F_body_dx(1:num-1,:)+F_body_dx(2:num,:)).*(l_i_dx(1:num-1)-l_i_dx(2:num))'./2,1);
M_sx_i=cross(P_sx,F_body_sx)+M_body_sx;
M_dx_i=cross(P_dx,F_body_dx)+M_body_dx;
M_sx=sum((M_sx_i(1:num-1,:)+M_sx_i(2:num,:)).*(l_i_sx(2:num)-l_i_sx(1:num-1))'./2,1);
M_dx=sum((M_dx_i(1:num-1,:)+M_dx_i(2:num,:)).*(l_i_dx(1:num-1)-l_i_dx(2:num))'./2,1);


% figure()
% plot3(I_origin_Dx(1),I_origin_Dx(2),I_origin_Dx(3),'or');
% hold on
% grid on
% plot3(P_dx(:,1),P_dx(:,2),P_dx(:,3),'oc');
% plot3(P_sx(:,1),P_sx(:,2),P_sx(:,3),'ob');
% quiver3(P_dx(:,1),P_dx(:,2),P_dx(:,3),V_dx(:,1),V_dx(:,2),V_dx(:,3));
% quiver3(P_sx(:,1),P_sx(:,2),P_sx(:,3),V_sx(:,1),V_sx(:,2),V_sx(:,3));
% quiver3(P_sx(:,1),P_sx(:,2),P_sx(:,3),L_aer_i(:,1),L_aer_i(:,2),L_aer_i(:,3),'r');
% quiver3(P_sx(:,1),P_sx(:,2),P_sx(:,3),F_body_sx(:,1),F_body_sx(:,2),F_body_sx(:,3),'b');
% axis equal

F_tot=-(F_sx+F_dx)
M_tot=-(M_sx+M_dx)
% CG=(Tl_0'*CG')';
% U
% Om
% [F_tot,M_tot]=AeroDynamics(U,Om,BoomInfo)
% norm(F_tot)
F_tot=Tl_0'*F_tot'
M_tot=Tl_0'*M_tot'
Us=Tl_0'*U';
Oms=Tl_0'*Om';

figure(1)
h2=quiver3(CG(1),CG(2),CG(3),F_tot(1),F_tot(2),F_tot(3),'r');
h3=quiver3(CG(1),CG(2),CG(3),M_tot(1),M_tot(2),M_tot(3),'b');
h4=quiver3(CG(1),CG(2),CG(3),Us(1),Us(2),Us(3),'g','linewidth',1);
h5=quiver3(CG(1),CG(2),CG(3),Oms(1)/100,Oms(2)/100,Oms(3)/100,'c');

legend([h1 h2 h3 h4 h5],{'Asse-Terreno','Force AERO','Momentum AERO','V','Om'});
