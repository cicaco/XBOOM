function [F,M,AoA2,Re]=AeroDynamics(u,omega)
% AeroDynamics computes the force and momentum for the whole boomerang
% and the angle of attack and Reynolds for each section spanwise
%
% [F,M,AoA2,Re]=AERODYNAMICS(u,omega) computes the quantity described as
% function of u and omega
%
% u velocity in the body frame
% omega angular speed in the body frame

%geometria
R=0.30; %m
c=0.0488; %m
xac1=0.0723; %va cambiato tra prima e seconda pala (DA FARE)

%spanwise wing distance
span1=linspace(0,R,20);
eta1=midspan(span1);
%calcolo n° Re
spanRe=3/4*R;

%sdr PALA j
sigma=120*pi/180;
coning=0; %tipo diedro (rot asse x2)
pitch=6*pi/180; %pitch della pala (rot asse y3)
%svergolamento
twist1=zeros(1,length(eta1));

%matrice di rotazione da body a blade
Tj1=[sin(sigma)*cos(pitch)+cos(sigma)*sin(coning)*sin(pitch), -cos(sigma)*cos(pitch)+sin(sigma)*sin(coning)*sin(pitch), -cos(coning)*sin(pitch)
    cos(sigma)*cos(coning), sin(sigma)*cos(coning), +sin(coning)
    sin(sigma)*sin(pitch)-cos(sigma)*sin(coning)*cos(pitch), -cos(sigma)*sin(pitch)-sin(sigma)*sin(coning)*cos(pitch), cos(coning)*cos(pitch)];

%SECONDA PALA
L=c/2/sin(pi-sigma);
xac2=0.0723+L; 
%spanwise wing distance
span2=linspace(sqrt(L^2-(c/2)^2),R+sqrt(L^2-(c/2)^2),20);
eta2=midspan(span2);
%sdr PALA 2
sigma=(240)*pi/180;
coning=0; %tipo diedro (rot asse x2)
pitch=6*pi/180; %pitch della pala (rot asse y3)
%svergolamento
twist2=zeros(1,length(eta2));

%matrice di rotazione da body a blade
Tj2=[sin(sigma)*cos(pitch)+cos(sigma)*sin(coning)*sin(pitch), -cos(sigma)*cos(pitch)+sin(sigma)*sin(coning)*sin(pitch), -cos(coning)*sin(pitch)
    cos(sigma)*cos(coning), sin(sigma)*cos(coning), +sin(coning)
    sin(sigma)*sin(pitch)-cos(sigma)*sin(coning)*cos(pitch), -cos(sigma)*sin(pitch)-sin(sigma)*sin(coning)*cos(pitch), cos(coning)*cos(pitch)];

%vel indotta
v_ind_old=0*3*0.5; %ipotesi iniziale, DA CAMBIARE

%inizializzazione
rA1=[]; v1=[]; w1=[]; AoA1=[]; FA1=[]; MA1=[];F1=[];M1=[];
rA2=[]; v2=[]; w2=[]; AoA2=[]; FA2=[]; MA2=[];F2=[];M2=[];
Re=[];
for i=1:length(eta1)
%blade element position in body frame
ra1=[xac1;0;0]+Tj1'*[0;eta1(i);0];
ra2=[xac2;0;0]+Tj2'*[0;eta2(i);0];
%velocity of blade element
vel1=u+cross(omega,ra1);
vel2=u+cross(omega,ra2);
%relative velocity of blade in blade frame
wel1= Tj1*(-vel1-[0;0;v_ind_old]);
wel2= Tj2*(-vel2-[0;0;v_ind_old]);
%AoA
aoa1=atan2(wel1(3),wel1(1))+twist1(i);
aoa2=atan2(wel2(3),wel2(1))+twist2(i);

%salvataggio variabili
rA1=[rA1, ra1];
rA2=[rA2, ra2];
% v1=[v1, vel1];
% v2=[v2, vel2];
w1=[w1, wel1];
w2=[w2, wel2];
AoA1=[AoA1, aoa1];
AoA2=[AoA2, aoa2];
end

%BET
for i=1:length(eta1)
    fa1=(span1(i+1)-span1(i))*0.5*1.225*c*norm(w1(:,i))^2*[-CL(AoA1(i))*sin(AoA1(i))+CD_new(AoA1(i))*cos(AoA1(i)); 0; CL(AoA1(i))*cos(AoA1(i))+CD_new(AoA1(i))*sin(AoA1(i))];
    ma1=(span1(i+1)-span1(i))*0.5*1.225*c*norm(w1(:,i))^2*[(CL(AoA1(i))*cos(AoA1(i))+CD_new(AoA1(i))*sin(AoA1(i)))*eta1(i); c*CM(AoA1(i)); (CL(AoA1(i))*sin(AoA1(i))-CD_new(AoA1(i))*cos(AoA1(i)))*eta1(i)];
    fa2=(span2(i+1)-span2(i))*0.5*1.225*c*norm(w2(:,i))^2*[-CL(AoA2(i))*sin(AoA2(i))+CD_new(AoA2(i))*cos(AoA2(i)); 0; CL(AoA2(i))*cos(AoA2(i))+CD_new(AoA2(i))*sin(AoA2(i))];
    ma2=(span2(i+1)-span2(i))*0.5*1.225*c*norm(w2(:,i))^2*[(CL(AoA2(i))*cos(AoA2(i))+CD_new(AoA2(i))*sin(AoA2(i)))*eta2(i); c*CM(AoA2(i)); (CL(AoA2(i))*sin(AoA2(i))-CD_new(AoA2(i))*cos(AoA2(i)))*eta2(i)];
    %forze su sdr body
    f1=(Tj1')*fa1;
    m1=(Tj1')*ma1+cross(rA1(:,i),(Tj1')*fa1);
    f2=Tj2'*fa2;
    m2=Tj2'*ma2+cross(rA2(:,i),Tj2'*fa2);
    
    %aggiornamento variabili
%     FA1=[FA1, fa1 ];
%     MA1=[MA1, ma1];
    F1=[F1,f1];
    M1=[M1,m1];
%     FA2=[FA2, fa2];
%     MA2=[MA2, ma2];
    F2=[F2, f2];
    M2=[M2, m2 ];
end

F1=sum(F1,2);
M1=sum(M1,2);

F2=sum(F2,2);
M2=sum(M2,2);

F=F1+F2;
M=M1+M2;

%% calcolo Re
ra1_Re=[xac1;0;0]+Tj1'*[0;spanRe;0];
vel1_Re=u+cross(omega,ra1_Re);
wel1_Re= Tj1*(-vel1_Re-[0;0;v_ind_old]);

Re=1.225*norm([wel1_Re(1), wel1_Re(2), wel1_Re(3)])*c/(1.81*10^-5);
