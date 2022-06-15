function [vel_ind]=vel_indotta_computation(u,omega,Tj1,Tj2,v_ind_old)

psi_ciclo=linspace(0,2*pi,20);
psi_vect=midspan(psi_ciclo);

%geometria
R=0.30; %m
c=0.0488; %m
xac1=0.0723;
%spanwise wing distance
span1=linspace(0,R,20);
eta1=midspan(span1);
%SECONDA PALA
L=c/2/sin(pi-120*pi/180);
xac2=0.0723;
%spanwise wing distance
span2=linspace(0,R,20);
eta2=midspan(span2);
F_ciclopsi=[];
M_ciclopsi=[];
for j=1:length(psi_vect)
    psi=psi_vect(j);
T_ciclopsi=[cos(psi), sin(psi), 0
    -sin(psi), cos(psi), 0
    0, 0, 1];

xac1_ciclopsi=T_ciclopsi'*[xac1; 0; 0];
xac2_ciclopsi=T_ciclopsi'*[xac2; 0; 0];

%inizializzazione
rA1=[]; v1=[]; w1=[]; AoA1=[]; FA1=[]; MA1=[];F1=[];M1=[];
rA2=[]; v2=[]; w2=[]; AoA2=[]; FA2=[]; MA2=[];F2=[];M2=[];
Re=[];
for i=1:length(eta1)
%blade element position in body frame
ra1=xac1_ciclopsi+Tj1'*[0;eta1(i);0];
ra2=xac2_ciclopsi+Tj2'*[0;eta2(i);0];
%velocity of blade element
vel1=u+cross(omega,ra1);
vel2=u+cross(omega,ra2);
%relative velocity of blade in blade frame
wel1= Tj1*(-vel1-[0;0;-v_ind_old]);
wel2= Tj2*(-vel2-[0;0;-v_ind_old]);
%AoA
aoa1=atan2(wel1(3),wel1(1));
aoa2=atan2(wel2(3),wel2(1));

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
    fa1=(span1(i+1)-span1(i))*0.5*1.225*c*norm(w1([1 3],i))^2*[-CL(AoA1(i))*sin(AoA1(i))+CD_new(AoA1(i))*cos(AoA1(i)); 0; CL(AoA1(i))*cos(AoA1(i))+CD_new(AoA1(i))*sin(AoA1(i))];
    ma1=(span1(i+1)-span1(i))*0.5*1.225*c*norm(w1([1 3],i))^2*[(CL(AoA1(i))*cos(AoA1(i))+CD_new(AoA1(i))*sin(AoA1(i)))*eta1(i); c*CM(AoA1(i)); (CL(AoA1(i))*sin(AoA1(i))-CD_new(AoA1(i))*cos(AoA1(i)))*eta1(i)];
    fa2=(span2(i+1)-span2(i))*0.5*1.225*c*norm(w2([1 3],i))^2*[-CL(AoA2(i))*sin(AoA2(i))+CD_new(AoA2(i))*cos(AoA2(i)); 0; CL(AoA2(i))*cos(AoA2(i))+CD_new(AoA2(i))*sin(AoA2(i))];
    ma2=(span2(i+1)-span2(i))*0.5*1.225*c*norm(w2([1 3],i))^2*[(CL(AoA2(i))*cos(AoA2(i))+CD_new(AoA2(i))*sin(AoA2(i)))*eta2(i); c*CM(AoA2(i)); (CL(AoA2(i))*sin(AoA2(i))-CD_new(AoA2(i))*cos(AoA2(i)))*eta2(i)];
    %forze su sdr body
    f1=(Tj1')*fa1;
    m1=(Tj1')*ma1+cross(xac1_ciclopsi,(Tj1')*fa1);
    f2=Tj2'*fa2;
    m2=Tj2'*ma2+cross(xac2_ciclopsi,(Tj2')*fa2);
    
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

F_ciclopsi=[F_ciclopsi F];
M_ciclopsi=[M_ciclopsi M];
end

%calcolo v_indotta
S=2.28*10^-1;
v_ind_new=0;
for j=1:length(psi_vect)
integral=1/(2*pi)*F_ciclopsi(3,j)/(2*1.225*S*sqrt(u(1)^2+(u(3)-v_ind_old)^2))*(psi_ciclo(j+1)-psi_ciclo(j));
v_ind_new=v_ind_new+integral;
end
vel_ind=v_ind_new;