function [F,M,AoA1,AoA2,Re,Mn]=AeroDynamics(u,omega,BoomInfo)
% AeroDynamics computes the force and momentum for the whole boomerang
% and the angle of attack and Reynolds for each section spanwise
%
% [F,M,AoA1,AoA2,Re,Mn]=AeroDynamics(u,omega,BoomInfo) computes the quantity described as
% function of u and omega
%
% INPUT:
% u velocity in the body frame
% omega angular speed in the body frame
%
% OUTPUT:
% F: vector of aerodynamic forces in body frame
% M: vector of aerodynamic moments in body frame
% AoAsx: Angles of Attack of each section of left blade
% AoAdx: Angles of Attack of each section of right blade
% Re: Reynolds number of a charateristic blade section at 3/4 Length of blade

%% import geometria
c=BoomInfo.Profile.Chord; %m
L=BoomInfo.Pianta.l;
freccia=BoomInfo.Pianta.freccia;
coning=BoomInfo.Pianta.diedro; %(rot asse x2)
pitch=BoomInfo.Pianta.pitch; %(rot asse y3)
%pala sx (denominata 1)
xac1=BoomInfo.Aero.P_origin_Sx(1); %posizione centro aerodinamico pala 1
start1=BoomInfo.Aero.Start_Sx; %inizio pala 1 con profilo costante
%pala2
xac2=BoomInfo.Aero.P_origin_Dx(1);%posizione centro aerodinamico pala 2
start2=BoomInfo.Aero.Start_Dx; %inizio pala 2 con profilo costante

%% definizione sistemi di riferimento pala
%PALA 1
span1=linspace(start1,start1+L,10);
eta1=midspan(span1);
lambda=pi/2+freccia;
%svergolamento
twist1=zeros(1,length(eta1));
% twist1=linspace(0,2*pi/180,length(eta1));

%spanwise wing distance

%calcolo n° Re
spanRe=3/4*(start1+L);

%matrice di rotazione da body a blade pala 1
Tj1=[sin(lambda)*cos(pitch)+cos(lambda)*sin(coning)*sin(pitch), -cos(lambda)*cos(pitch)+sin(lambda)*sin(coning)*sin(pitch), -cos(coning)*sin(pitch)
    cos(lambda)*cos(coning), sin(lambda)*cos(coning), +sin(coning)
    sin(lambda)*sin(pitch)-cos(lambda)*sin(coning)*cos(pitch), -cos(lambda)*sin(pitch)-sin(lambda)*sin(coning)*cos(pitch), cos(coning)*cos(pitch)];


%SECONDA PALA
%spanwise wing distance
span2=linspace(start2,start2+L,10);
eta2=midspan(span2);
lambda=2*pi-lambda;
%svergolamento
twist2=zeros(1,length(eta2));
% twist2=linspace(0,2*pi/180,length(eta1));
%matrice di rotazione da body a blade
Tj2=[sin(lambda)*cos(pitch)+cos(lambda)*sin(coning)*sin(pitch), -cos(lambda)*cos(pitch)+sin(lambda)*sin(coning)*sin(pitch), -cos(coning)*sin(pitch)
    cos(lambda)*cos(coning), sin(lambda)*cos(coning), +sin(coning)
    sin(lambda)*sin(pitch)-cos(lambda)*sin(coning)*cos(pitch), -cos(lambda)*sin(pitch)-sin(lambda)*sin(coning)*cos(pitch), cos(coning)*cos(pitch)];
%% calcoli aerodinamici
%vel indotta
v_ind_old=0; %ipotesi iniziale
err=1;
while err>10^-3
[vel_ind]=vel_indotta_computation(u,omega,v_ind_old,BoomInfo);
err=abs((vel_ind-v_ind_old)/vel_ind);
v_ind_old=vel_ind;
end
% inizializzazione
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
% rA1=[rA1, ra1];
% rA2=[rA2, ra2];
% v1=[v1, vel1];
% v2=[v2, vel2];
w1=[w1, wel1];
w2=[w2, wel2];
AoA1=[AoA1, aoa1];
AoA2=[AoA2, aoa2];
end

%BET
for i=1:length(eta1)
    fa1=(span1(i+1)-span1(i))*0.5*1.225*c*norm(w1([1 3],i))^2*[-CL(AoA1(i))*sin(AoA1(i))+CD(AoA1(i))*cos(AoA1(i)); 0; CL(AoA1(i))*cos(AoA1(i))+CD(AoA1(i))*sin(AoA1(i))];
    ma1=(span1(i+1)-span1(i))*0.5*1.225*c*norm(w1([1 3],i))^2*[(CL(AoA1(i))*cos(AoA1(i))+CD(AoA1(i))*sin(AoA1(i)))*eta1(i); c*CM(AoA1(i)); (CL(AoA1(i))*sin(AoA1(i))-CD(AoA1(i))*cos(AoA1(i)))*eta1(i)];
    fa2=(span2(i+1)-span2(i))*0.5*1.225*c*norm(w2([1 3],i))^2*[-CL(AoA2(i))*sin(AoA2(i))+CD(AoA2(i))*cos(AoA2(i)); 0; CL(AoA2(i))*cos(AoA2(i))+CD(AoA2(i))*sin(AoA2(i))];
    ma2=(span2(i+1)-span2(i))*0.5*1.225*c*norm(w2([1 3],i))^2*[(CL(AoA2(i))*cos(AoA2(i))+CD(AoA2(i))*sin(AoA2(i)))*eta2(i); c*CM(AoA2(i)); (CL(AoA2(i))*sin(AoA2(i))-CD(AoA2(i))*cos(AoA2(i)))*eta2(i)];

%      fa1=(span1(i+1)-span1(i))*0.5*1.225*c*norm(w1([1 3],i))^2*[-CL_naca(AoA1(i))*sin(AoA1(i))+CD_naca(AoA1(i))*cos(AoA1(i)); 0; CL_naca(AoA1(i))*cos(AoA1(i))+CD_naca(AoA1(i))*sin(AoA1(i))];
%      ma1=(span1(i+1)-span1(i))*0.5*1.225*c*norm(w1([1 3],i))^2*[(CL_naca(AoA1(i))*cos(AoA1(i))+CD_naca(AoA1(i))*sin(AoA1(i)))*eta1(i); c*CM_naca(AoA1(i)); (CL_naca(AoA1(i))*sin(AoA1(i))-CD_naca(AoA1(i))*cos(AoA1(i)))*eta1(i)];
%      fa2=(span2(i+1)-span2(i))*0.5*1.225*c*norm(w2([1 3],i))^2*[-CL_naca(AoA2(i))*sin(AoA2(i))+CD_naca(AoA2(i))*cos(AoA2(i)); 0; CL_naca(AoA2(i))*cos(AoA2(i))+CD_naca(AoA2(i))*sin(AoA2(i))];
%      ma2=(span2(i+1)-span2(i))*0.5*1.225*c*norm(w2([1 3],i))^2*[(CL_naca(AoA2(i))*cos(AoA2(i))+CD_naca(AoA2(i))*sin(AoA2(i)))*eta2(i); c*CM_naca(AoA2(i)); (CL_naca(AoA2(i))*sin(AoA2(i))-CD_naca(AoA2(i))*cos(AoA2(i)))*eta2(i)];
   
    %forze su sdr body
    f1=(Tj1')*fa1;
    m1=(Tj1')*ma1+cross([xac1;0;0],(Tj1')*fa1);
    f2=Tj2'*fa2;
    m2=Tj2'*ma2+cross([xac2;0;0],Tj2'*fa2);
    
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

Re=1.225*norm([wel1_Re(1), 0, wel1_Re(3)])*c/(1.81*10^-5);
%% Calcolo momento Mn
vn_versore=-[u(1) u(2) 0]./norm([u(1) u(2) 0]);
Mn=vn_versore*M;
