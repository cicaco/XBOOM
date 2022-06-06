clear all
clc
close all
Chord=0.0488;
Profile2D=importdata('Naca0012.dat');

Xp=-[Profile2D.data(2:67,1) ; fliplr(Profile2D.data(68:end,1)')'].*Chord;
Yp=zeros(length(Xp),1);
Zp=[Profile2D.data(2:67,2) ; fliplr(Profile2D.data(68:end,2)')'].*Chord;

Xp_flip=-(Chord/2.*ones(size(Xp))+Xp)+Chord/2.*ones(size(Xp))-0.25*Chord;
Zp_flip=(Zp);

Xp=Xp_flip';Yp=Yp';Zp=Zp';
figure(10)
plot3(Xp,Yp,Zp)

figure(1)
hold on
axis equal
Profilo=[Xp;Yp;Zp];
%rotazione pala sx
%sdr PALA j
sigma=120*pi/180;
coning=0*pi/180; %tipo diedro (rot asse x2)
pitch=0*pi/180; %pitch della pala (rot asse y3)

%matrice di rotazione da body a blade
Tj1=[sin(sigma)*cos(pitch)+cos(sigma)*sin(coning)*sin(pitch), -cos(sigma)*cos(pitch)+sin(sigma)*sin(coning)*sin(pitch), -cos(coning)*sin(pitch)
    cos(sigma)*cos(coning), sin(sigma)*cos(coning), +sin(coning)
    sin(sigma)*sin(pitch)-cos(sigma)*sin(coning)*cos(pitch), -cos(sigma)*sin(pitch)-sin(sigma)*sin(coning)*cos(pitch), cos(coning)*cos(pitch)];

Profilo_ruot=Tj1'*Profilo;


eta=linspace(0.25*Chord*tan(sigma-pi/2),0.3+0.25*Chord*tan(sigma-pi/2),5);
P_l=[];
for i=1:5
%traslo profilo
xac1=0.0;
ra1=[xac1;0;0]+Tj1'*[0;eta(i);0];
Profilo_trasl_left=ra1+Profilo_ruot;
P_l=[P_l;Profilo_trasl_left'];
figure(1)
hold on
plot3(Profilo_trasl_left(1,:),Profilo_trasl_left(2,:),Profilo_trasl_left(3,:),'*g');
xlabel('X');
ylabel('Y')
end

%% seconda pala
Profile2D=importdata('Naca0012.dat');

Xp=-[Profile2D.data(2:67,1) ; fliplr(Profile2D.data(68:end,1)')'].*Chord;
Yp=zeros(length(Xp),1);
Zp=[Profile2D.data(2:67,2) ; fliplr(Profile2D.data(68:end,2)')'].*Chord;
Xp_flip=-(Chord/2.*ones(size(Xp))+Xp)+Chord/2.*ones(size(Xp))-0.25*Chord;
Zp_flip=(Zp);

Xp=Xp_flip';Yp=Yp';Zp=Zp';
Profilo=[Xp;Yp;Zp];
%rotazione pala sx
%sdr PALA j
sigma=240*pi/180;
coning=0*pi/180; %tipo diedro (rot asse x2)
pitch=0*pi/180; %pitch della pala (rot asse y3)

%matrice di rotazione da body a blade
Tj2=[sin(sigma)*cos(pitch)+cos(sigma)*sin(coning)*sin(pitch), -cos(sigma)*cos(pitch)+sin(sigma)*sin(coning)*sin(pitch), -cos(coning)*sin(pitch)
    cos(sigma)*cos(coning), sin(sigma)*cos(coning), +sin(coning)
    sin(sigma)*sin(pitch)-cos(sigma)*sin(coning)*cos(pitch), -cos(sigma)*sin(pitch)-sin(sigma)*sin(coning)*cos(pitch), cos(coning)*cos(pitch)];

Profilo_ruot=Tj2'*Profilo;
figure(11)
plot3(Profilo_ruot(1,:),Profilo_ruot(2,:),Profilo_ruot(3,:))
hold on
plot3(Profilo(1,:),Profilo(2,:),Profilo(3,:))

L=0.5*Chord/cos(2*pi-pi/2-sigma);
eta=linspace(3/4*Chord*tan(2*pi-pi/2-sigma),0.3+3/4*Chord*tan(2*pi-pi/2-sigma),5);
% eta=[0]
P_r=[];
for i=1:5
%traslo profilo

xac2=L;
ra2=[xac2;0;0]+Tj2'*[0;eta(i);0];
Profilo_trasl_right=ra2+Profilo_ruot;
P_r=[P_r; Profilo_trasl_right'];
figure(1)
hold on
plot3(Profilo_trasl_right(1,:),Profilo_trasl_right(2,:),Profilo_trasl_right(3,:),'*b');
xlabel('X');
ylabel('Y')
axis equal
end

%% Effettuo la triangolazione
dt = delaunayTriangulation(P_l);
[tri_l, Xb_l]=freeBoundary(dt);
dt = delaunayTriangulation(P_r);
[tri_r, Xb_r]=freeBoundary(dt);
n=length(Xb_l);
pr=triangulation([tri_l;tri_r+n],[Xb_l;Xb_r]);
% pr=triangulation([tri_r],[Xb_r]);
stlwrite(pr, 'provaSimo.stl');
%%
addpath(genpath('3DSolidGeneration'));


RBP=RigidBodyParams(pr);
disp(RBP)
VisualizeLocalFrame(pr)

CG=RBP.centroid;

Xb_l_CG=[Xb_l(:,1)-CG(1) Xb_l(:,2)-CG(2) Xb_l(:,3)-CG(3)];
Xb_r_CG=[Xb_r(:,1)-CG(1) Xb_r(:,2)-CG(2)  Xb_r(:,3)-CG(3)];
pr_CG=triangulation([tri_l;tri_r+n],[Xb_l_CG;Xb_r_CG]);
RBP_CG=RigidBodyParams(pr_CG);
m=0.13
Rho=m/RBP_CG.volume;
I=Rho*RBP_CG.inertia_tensor

CG=RBP_CG.centroid;

%% test su asta
theta=linspace(0,2*pi,50);
x=cos(theta);
y=sin(theta);
z=zeros(1,50);
ASTA=[x'  y' z'];
for i=1:100
    z=i.*ones(1,50);
    ASTA=[ASTA; x' y' z'];
end

dt = delaunayTriangulation(ASTA);
% [tri_r, Xb_r]=freeBoundary(dt);

pr=triangulation([tri_r],[Xb_r]);
% pr=triangulation([tri_r],[Xb_r]);
stlwrite(pr, 'provaSimo.stl');
RBP=RigidBodyParams(pr);
disp(RBP)
VisualizeLocalFrame(pr)

CG=RBP.centroid;


m=1;
Rho=m/RBP.volume;
I=Rho*RBP.inertia_tensor
