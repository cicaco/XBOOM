clear all
close all
addpath(genpath('3DSolidGeneration'));

%% Input Data
Chord=0.0488;
p_c=20; % Tale valore decide da dove deve partire la trasizione
p_t=5;
l=0.3;
delta=30*pi/180;
beta=0*pi/180;
pitch=0*pi/180;
num=40;
PARA=1.5;
%% DATA
% Profile 2D Shape
Profile2D=importdata('Naca0012.dat');
%Essenziale mettere due volte lo zero!!
%Sistemo il plot in senso orario partendo da 0,0
Xp=-[Profile2D.data(2:67,1) ; fliplr(Profile2D.data(68:end,1)')'].*Chord;
Zp=[Profile2D.data(2:67,2) ; fliplr(Profile2D.data(68:end,2)')'].*Chord;
Xp_flip=-(Chord/2.*ones(size(Xp))+Xp)+Chord/2.*ones(size(Xp))-Chord;
Zp_flip=(Zp);
figure()
plot(Xp,Zp,'*r');
hold on
plot(Xp_flip,Zp_flip,'oc');
axis equal
set(gca,'Xdir','reverse')
[n,~]=size(Xp);
%Clock-wise direction regeneration
Xp_flip=[fliplr(Xp_flip(1:n/2)')';fliplr(Xp_flip(n/2+1:end)')'];
Zp_flip=[fliplr(Zp_flip(1:n/2)')';fliplr(Zp_flip(n/2+1:end)')'];
%%
C=linspace(0,-Chord,num);
figure(1)
plot(C,zeros(1,num),'*k');
hold on
grid on
U_Delta=[0 0 1];
%% corda che deve essere traslata e ruotata
P=[C;zeros(1,num);zeros(1,num)];
[X_trans,Z_trans] = Profile2d_Trans(Xp,Zp_flip,Zp,p_c*2-1);
X_2d_i=[ones(num-p_c,1)*Xp';X_trans;ones(num-p_c,1)*Xp_flip'];
Z_2d_i=[ones(num-p_c,1)*Zp';Z_trans;ones(num-p_c,1)*Zp_flip'];



D_i=[-delta.*ones(1,num-p_c) linspace(-delta,delta,2*p_c-1) delta.*ones(1,num-p_c)];
B_i_aer=[-beta.*ones(1,num) beta.*ones(1,num-1)];
D_i_aer=[-delta.*ones(1,num) delta.*ones(1,num-1)];
Fract=[1/4*ones(1,num-p_c) linspace(1/4,3/4,2*p_c-1) 3/4*ones(1,num-p_c)];
B_i=[-beta.*ones(1,num-p_c) linspace(-beta,beta,2*p_c-1) beta.*ones(1,num-p_c)];
P_i=[-pitch.*ones(1,num-p_c) linspace(-pitch,pitch,2*p_c-1) pitch.*ones(1,num-p_c)];
P_i=linspace(-pitch,pitch,2*num-1);

% Caso con matrici rotazione
%  D_i=[pi/2-delta.*ones(1,num-p_c) linspace(3/2*pi-delta,pi/2+delta,2*p_c-1) pi/2+delta.*ones(1,num-p_c)];
%  D_i_aer=[3/2*pi-delta.*ones(1,num) pi/2+delta.*ones(1,num-1)];

B_i_aer=[-beta.*ones(1,num-p_c) linspace(-beta,beta,2*p_c-1) beta.*ones(1,num-p_c)];

li=linspace(l,0,num-p_c);
Tx=[ -sin(delta).*li zeros(1,p_c*2-1) -sin(delta).*li];
Ty=[ -cos(delta).*li zeros(1,p_c*2-1) cos(delta).*li];
Tz=zeros(1,numel(Tx));
P_r=[];
P_center_r=[];

figure()
for i=1:num
    
    [P_1] = Rot_Point([0 0 1],D_i(i),P,[C(end)*PARA;0;0]); %Rotazione rispetto all'asse z, rispetto però al "bordo di uscita" del profilo
    P_1(1,:)=P_1(1,:)+Tx(i);
    P_1(2,:)=P_1(2,:)+Ty(i);
    [C_aer] = AerCenter(P_1,Chord,Fract(i));
    plot(P_1(1,:),P_1(2,:),'r');
    hold on
    U_beta= Rot_Point(U_Delta,D_i_aer(i),[1; 0 ;0],[Chord 0 0]'); %devo ottenere il versore ruotato da x a x2 (body)
    [C_aer_rot] = Rot_Point(U_beta,B_i_aer(i),C_aer,[0;0;0]);
    
    C_fin(:,i)=C_aer;
    C_fin_rot(:,i)=C_aer_rot;
    %Adesso devo ruotare la pala
end
set(gca,'Ydir','reverse')
set(gca,'Xdir','reverse')
axis equal
figure()
C_fin_rot(2,end)=0;
plot3(C_fin_rot(1,:),C_fin_rot(2,:),C_fin_rot(3,:),'ob');
hold on
plot3(C_fin(1,:),C_fin(2,:),C_fin(3,:),'*r');

set(gca,'Ydir','reverse')
set(gca,'Xdir','reverse')
axis equal
%%
P_tot=[];
for i=1:num%num*2-1
    %Ruoto il profilo sempre rispetto al centro aerodinamico
    P_2d=[X_2d_i(i,:);zeros(size(Z_2d_i(i,:)));Z_2d_i(i,:)];
    %P_2d=P;
    [P_fin] = Rot_Point([0 0 1],D_i(i),P_2d,[Fract(i)*C(end);0;0]); %Rotazione rispetto all'asse z, rispetto però al "bordo di uscita" del profilo
    U_beta= Rot_Point(U_Delta,D_i(i),[1; 0 ;0],[Fract(i)*C(end);0;0]); %devo ottenere il versore ruotato da x a x2 (body)
    [P_fin] = Rot_Point(U_beta,B_i(i),P_fin,[Fract(i)*C(end);0;0]);
    U_pitch_d= Rot_Point(U_Delta,D_i(i),[0; 1; 0 ],[Fract(i)*C(end);0;0]); % devo ottenre il versore ruotato da [0 1 0], ruotato di delta e poi di beta
    U_pitch= Rot_Point(U_beta,B_i(i),U_pitch_d,[Fract(i)*C(end);0;0]);
    %[C_aer] = AerCenter(P_fin,i,num,Chord);
    [P_fin] = Rot_Point(U_pitch ,P_i(i),P_fin,[Fract(i)*C(end);0;0]);
    
    % Fino a num sono sulla pala destra e il centro aerodinamico e c/4
%     sigma=D_i(i);
%     coning=B_i(i);
%     pitch=P_i(i);
%     Tj1=[sin(sigma)*cos(pitch)+cos(sigma)*sin(coning)*sin(pitch), -cos(sigma)*cos(pitch)+sin(sigma)*sin(coning)*sin(pitch), -cos(coning)*sin(pitch)
%     cos(sigma)*cos(coning), sin(sigma)*cos(coning), +sin(coning)
%     sin(sigma)*sin(pitch)-cos(sigma)*sin(coning)*cos(pitch), -cos(sigma)*sin(pitch)-sin(sigma)*sin(coning)*cos(pitch), cos(coning)*cos(pitch)];

%     P_fin=Tj1'*P_2d;

    P_fin(1,:)=P_fin(1,:)-Fract(i)*C(end)+C_fin_rot(1,i);
    P_fin(2,:)=P_fin(2,:)+C_fin_rot(2,i);
    P_fin(3,:)=P_fin(3,:)+C_fin_rot(3,i);
    hold on
    if i<=num-p_c
        
        P_r=[P_r;P_fin'];
    elseif  i>=num-p_c && i<=num+p_c
        P_center_r=[P_center_r;P_fin'];
    else
        P_l=[P_l;P_fin'];
    end
    if i==num-p_c
        P_prova=P_fin;
        P_add=(P_fin(:,1:66)+fliplr(P_fin(:,67:end)))'./2;
        P_center_r=[P_center_r;P_add];
    end
    P_tot=[P_tot;P_fin];
end
figure()
plot3(P_center_r(:,1),P_center_r(:,2),P_center_r(:,3),'*r');
hold on
quiver3(0,0,0,l*U_pitch(1),l*U_pitch(2),l*U_pitch(3),'g')
quiver3(0,0,0,l*U_beta(1),l*U_beta(2),l*U_beta(3),'k')
%plot3(P_l(:,1),P_l(:,2),P_l(:,3),'*c');

%plot3(C_aer(:,1),C_aer(:,2),C_aer(:,3),'*b');
set(gca,'Ydir','reverse')
set(gca,'Xdir','reverse')
axis equal

% Effettuo la triangolazione
% dt = delaunayTriangulation(P_l);
% [tri_l, Xb_l]=freeBoundary(dt);
shp = alphaShape(P_r,1);
[tri_r, xyz_r] = boundaryFacets(shp);
n=length(xyz_r);
shp = alphaShape(P_center_r,'HoleThreshold',20);
figure()
plot(shp)
[tri_rc, xyz_rc] = boundaryFacets(shp);
figure(10)
trisurf(tri_r,xyz_r(:,1),xyz_r(:,2),xyz_r(:,3),...
    'FaceColor','cyan','FaceAlpha',0.3)
hold on
trisurf(tri_rc,xyz_rc(:,1),xyz_rc(:,2),xyz_rc(:,3),...
    'FaceColor','cyan','FaceAlpha',0.3)
axis equal
% n=length(Xb_l);
pr=triangulation([tri_r;tri_rc+n],[xyz_r;xyz_rc]);
%pr=triangulation([tri_r],[Xb_r]);
stlwrite(pr, 'Pala_dx.stl');

%%

%%
P_l=[];
P_center_l=[];
figure()
for i=num+1:2*num-1
    
    [P_1] = Rot_Point([0 0 1],D_i(i),P,[C(end)*PARA;0;0]); %Rotazione rispetto all'asse z, rispetto però al "bordo di uscita" del profilo
    P_1(1,:)=P_1(1,:)+Tx(i);
    P_1(2,:)=P_1(2,:)+Ty(i);
    [C_aer] = AerCenter(P_1,Chord,Fract(i));
    plot(P_1(1,:),P_1(2,:),'r');
    hold on
    U_beta= Rot_Point(U_Delta,D_i_aer(i),[1; 0 ;0],[Chord 0 0]'); %devo ottenere il versore ruotato da x a x2 (body)
    [C_aer_rot] = Rot_Point(U_beta,B_i_aer(i),C_aer,[0;0;0]);
    
    C_fin(:,i)=C_aer;
    C_fin_rot(:,i)=C_aer_rot;
    %Adesso devo ruotare la pala
end
set(gca,'Ydir','reverse')
set(gca,'Xdir','reverse')
axis equal
figure()
plot3(C_fin_rot(1,num:end),C_fin_rot(2,num:end),C_fin_rot(3,num:end),'ob');
hold on
plot3(C_fin(1,num:end),C_fin(2,num:end),C_fin(3,num:end),'*r');

set(gca,'Ydir','reverse')
set(gca,'Xdir','reverse')
axis equal
%%
for i=num:num*2-1
    %Ruoto il profilo sempre rispetto al centro aerodinamico
    P_2d=[X_2d_i(i,:);zeros(size(Z_2d_i(i,:)));Z_2d_i(i,:)];
    %P_2d=P;
    [P_fin] = Rot_Point([0 0 1],D_i(i),P_2d,[Fract(i)*C(end);0;0]); %Rotazione rispetto all'asse z, rispetto però al "bordo di uscita" del profilo
    U_beta= Rot_Point(U_Delta,D_i(i),[1; 0 ;0],[Fract(i)*C(end);0;0]); %devo ottenere il versore ruotato da x a x2 (body)
    [P_fin] = Rot_Point(U_beta,B_i(i),P_fin,[Fract(i)*C(end);0;0]);
    U_pitch_d= Rot_Point(U_Delta,D_i(i),[0; 1; 0 ],[Fract(i)*C(end);0;0]); % devo ottenre il versore ruotato da [0 1 0], ruotato di delta e poi di beta
    U_pitch= Rot_Point(U_beta,B_i(i),U_pitch_d,[Fract(i)*C(end);0;0]);
    %[C_aer] = AerCenter(P_fin,i,num,Chord);
    [P_fin] = Rot_Point(U_pitch ,P_i(i),P_fin,[Fract(i)*C(end);0;0]);
    
    % Fino a num sono sulla pala destra e il centro aerodinamico e c/4
%          sigma=D_i(i);
%      coning=B_i(i);
%      pitch=P_i(i);
%      Tj1=[sin(sigma)*cos(pitch)+cos(sigma)*sin(coning)*sin(pitch), -cos(sigma)*cos(pitch)+sin(sigma)*sin(coning)*sin(pitch), -cos(coning)*sin(pitch)
%      cos(sigma)*cos(coning), sin(sigma)*cos(coning), +sin(coning)
%      sin(sigma)*sin(pitch)-cos(sigma)*sin(coning)*cos(pitch), -cos(sigma)*sin(pitch)-sin(sigma)*sin(coning)*cos(pitch), cos(coning)*cos(pitch)];
%  
%      P_fin=Tj1'*P_2d;

    P_fin(1,:)=P_fin(1,:)-Fract(i)*C(end)+C_fin_rot(1,i);
    P_fin(2,:)=P_fin(2,:)+C_fin_rot(2,i);
    P_fin(3,:)=P_fin(3,:)+C_fin_rot(3,i);
    hold on
    if i>=num+p_c
        
        P_l=[P_l;P_fin'];
    else
        P_center_l=[P_center_l;P_fin'];
    
        
    end
    if i>num
     P_tot=[P_tot;P_fin];
    end
end
figure()
plot3(P_l(:,1),P_l(:,2),P_l(:,3),'*r');
hold on
quiver3(0,0,0,l*U_pitch(1),l*U_pitch(2),l*U_pitch(3),'g')
quiver3(0,0,0,l*U_beta(1),l*U_beta(2),l*U_beta(3),'k')
%plot3(P_l(:,1),P_l(:,2),P_l(:,3),'*c');

%plot3(C_aer(:,1),C_aer(:,2),C_aer(:,3),'*b');
set(gca,'Ydir','reverse')
set(gca,'Xdir','reverse')
axis equal

% Effettuo la triangolazione
% dt = delaunayTriangulation(P_l);
% [tri_l, Xb_l]=freeBoundary(dt);
shp = alphaShape(P_l,1);
[tri_l, xyz_l] = boundaryFacets(shp);
n=length(xyz_l);
shp = alphaShape(P_center_l,0.5,'HoleThreshold',0.02);
[tri_lc, xyz_lc] = boundaryFacets(shp);
tri = delaunay(P_center_l);
figure(72)

trimesh(tri, P_center_l(:,1), P_center_l(:,2), P_center_l(:,3),'EdgeColor','k');
figure(10)
trisurf(tri_l,xyz_l(:,1),xyz_l(:,2),xyz_l(:,3),...
    'FaceColor','red','FaceAlpha',0.3)
hold on
trisurf(tri_lc,xyz_lc(:,1),xyz_lc(:,2),xyz_lc(:,3),...
    'FaceColor','red','FaceAlpha',0.3)
axis equal
% n=length(Xb_l);
pr=triangulation([tri_l;tri_lc+n],[xyz_l;xyz_lc]);
%pr=triangulation([tri_r],[Xb_r]);
stlwrite(pr, 'Pala_sx.stl');
nr=length(xyz_r);
nrc=length(xyz_rc);
nlc=length(xyz_lc);
tri_tot=[tri_r;tri_rc+nr;tri_lc+nr+nrc;tri_l+nr+nrc+nlc];
xyz_tot=[xyz_r;xyz_rc;xyz_lc;xyz_l];
pr_fin=triangulation(tri_tot,xyz_tot);
stlwrite(pr_fin, 'Boom.stl');
% %%
%  RBP=RigidBodyParams(pr_fin);
%  CG=RBP.centroid;
%  
%  xyz_tot_CG=[xyz_tot(:,1)-CG(1) xyz_tot(:,2)-CG(2) xyz_tot(:,3)-CG(3)];
%  pr_CG=triangulation(tri_tot,xyz_tot_CG);
%  RBP_CG=RigidBodyParams(pr_CG);
%  m=0.13
%  Rho=m/RBP_CG.volume;
%  I=Rho*RBP_CG.inertia_tensor
%  
%  CG=RBP_CG.centroid;
%  %% calcolo inerzia rispetto agli assi baricentrali
% stlwrite(pr_CG, 'prova.stl');
%Create the complete solid
save('Profile3D','P_tot','p_c','num');

clear all
close all

figure()
for i=1:2*num-1
    plot3(P_tot(3*i-2,:),P_tot(3*i-1,:),P_tot(3*i,:),'*r');
    hold on
    axis equal
end
% Start assembly each solid
figure()
n_prec=0;
tr=[];
xyz=[];
for i=2:2*num-1
    P_prec=P_tot(3*(i-1)-2:3*(i-1),:);
    P_succ=P_tot(3*i-2:3*i,:);
    P_i=[P_prec';P_succ'];
    shp = alphaShape(P_i,1);
    [tr_i, xyz_i] = boundaryFacets(shp);
    n_succ=length(xyz_i)+n_prec;
        tr=[tr;tr_i+n_prec];
    xyz=[xyz;xyz_i];
    n_prec=n_succ;
    plot(shp)
    hold on
end

pr_fin=triangulation(tr,xyz);
stlwrite(pr_fin, 'Boom.stl');