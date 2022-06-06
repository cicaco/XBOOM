function [F,M,AoA1]=aero(u,omega)
%u velocità in sdr body
%omega velocità angolari in sdr body

%geometria
R=0.30; %m
c=0.0488; %m
xac=0.0723; %va cambiato tra prima e seconda pala (DA FARE)

%sdr PALA j
sigma=120*pi/180;
coning=0; %tipo diedro (rot asse x2)
pitch=0; %pitch della pala (rot asse y3)

%matrice di rotazione da body a blade
Tj=[sin(sigma)*cos(pitch)+cos(sigma)*sin(coning)*sin(pitch), -cos(sigma)*cos(pitch)+sin(sigma)*sin(coning)*sin(pitch), -cos(coning)*sin(pitch)
    cos(sigma)*cos(coning), sin(sigma)*cos(coning), +sin(coning)
    sin(sigma)*sin(pitch)-cos(sigma)*sin(coning)*cos(pitch), -cos(sigma)*sin(pitch)-sin(sigma)*sin(coning)*cos(pitch), cos(coning)*cos(pitch)];

%  Tj'*Tj
%spanwise wing distance
eta=linspace(0,R,15);

%blade element position in body frame
rA1=[];
for i=1:length(eta)
rA1=[rA1, [xac;0;0]+Tj'*[0;eta(i);0]];
end

%velocity of blade element
v1=[];
for i=1:length(eta)
v1=[v1, u+cross(omega,rA1(:,i))];
end

v_ind_old=0.00; %ipotesi iniziale, DA CAMBIARE
% err=1;
% while err>10^-3
    
%relative velocity of blade in blade frame
w1=[];
for i=1:length(eta)
w1=[w1, Tj*(-v1(:,i)-[0;0;v_ind_old])];
end

%AoA
AoA1=[];
%svergolamento
twist=zeros(1,length(eta));
for i=1:length(eta)
AoA1=[AoA1, atan2(w1(3,i),w1(1,i))+twist(i)];
end

%BET
FA1=[];
MA1=[];
for i=2:length(eta)
FA1=[FA1, (eta(i)-eta(i-1))*0.5*1.225*c*norm(w1(:,i))^2*[-CL(AoA1(i))*sin(AoA1(i))+CD_new(AoA1(i))*cos(AoA1(i)); 0; CL(AoA1(i))*cos(AoA1(i))+CD_new(AoA1(i))*sin(AoA1(i))]];
MA1=[MA1,(eta(i)-eta(i-1))*0.5*1.225*c*norm(w1(:,i))^2*[(CL(AoA1(i))*cos(AoA1(i))+CD_new(AoA1(i))*sin(AoA1(i)))*eta(i); c*CM(AoA1(i)); (CL(AoA1(i))*sin(AoA1(i))-CD_new(AoA1(i))*cos(AoA1(i)))*eta(i)] ];
end

%forze su sdr body
F1=[];
M1=[];
for i=1:length(FA1)
    F1=[F1, (Tj')*FA1(:,i)];
    M1=[M1, (Tj')*MA1(:,i)+cross(0.5*(rA1(:,i+1)+rA1(:,i)),(Tj')*FA1(:,i))];
end
% Tj'*Tj


F1=sum(F1,2);
M1=sum(M1,2);

%SECONDA PALA
xac=0.0723+c/2; %va cambiato tra prima e seconda pala (DA FARE)

%sdr PALA j
sigma=(240)*pi/180;
coning=0; %tipo diedro (rot asse x2)
pitch=0; %pitch della pala (rot asse y3)

%matrice di rotazione da body a blade
Tj=[sin(sigma)*cos(pitch)+cos(sigma)*sin(coning)*sin(pitch), -cos(sigma)*cos(pitch)+sin(sigma)*sin(coning)*sin(pitch), -cos(coning)*sin(pitch)
    cos(sigma)*cos(coning), sin(sigma)*cos(coning), +sin(coning)
    sin(sigma)*sin(pitch)-cos(sigma)*sin(coning)*cos(pitch), -cos(sigma)*sin(pitch)-sin(sigma)*sin(coning)*cos(pitch), cos(coning)*cos(pitch)];

%spanwise wing distance
eta=linspace(0,R,15);

%blade element position in body frame
rA2=[];
for i=1:length(eta)
rA2=[rA2, [xac;0;0]+Tj'*[0;eta(i);0]];
end

%velocity of blade element
v2=[];
for i=1:length(eta)
v2=[v2, u+cross(omega,rA2(:,i))];
end

v_ind=0; %ipotesi iniziale, DA CAMBIARE

%relative velocity of blade in blade frame
w2=[];
for i=1:length(eta)
w2=[w2, Tj*(-v2(:,i)-v_ind_old)];
end

%AoA
AoA2=[];
%svergolamento
twist=zeros(1,length(eta));
for i=1:length(eta)
AoA2=[AoA2, atan2(w2(3,i),w2(1,i))+twist(i)];
end

%BET
FA2=[];
MA2=[];
for i=2:length(eta)
FA2=[FA2, (eta(i)-eta(i-1))*0.5*1.225*c*norm(w2(:,i))^2*[-CL(AoA2(i))*sin(AoA2(i))+CD_new(AoA2(i))*cos(AoA2(i)); 0; CL(AoA2(i))*cos(AoA2(i))+CD_new(AoA2(i))*sin(AoA2(i))]];
MA2=[MA2,(eta(i)-eta(i-1))*0.5*1.225*c*norm(w2(:,i))^2*[(CL(AoA2(i))*cos(AoA2(i))+CD_new(AoA2(i))*sin(AoA2(i)))*eta(i); c*CM(AoA2(i)); (CL(AoA2(i))*sin(AoA2(i))-CD_new(AoA2(i))*cos(AoA2(i)))*eta(i)] ];
end

%forze su sdr body
F2=[];
M2=[];
for i=1:length(FA2)
    F2=[F2, Tj'*FA2(:,i)];
    M2=[M2, Tj'*MA2(:,i)+cross(0.5*(rA2(:,i+1)+rA2(:,i)),Tj'*FA2(:,i))];
end
F2=sum(F2,2);
M2=sum(M2,2);

F=F1+F2;
M=M1+M2;

% v_ind_new=(F(3)/(2*1.225*2.76*2*10^-2)/sqrt(u(1)^2+(u(3)-v_ind_old)^2)); 
% err=abs(v_ind_new-v_ind_old);
% v_ind_old=v_ind_new;
% end
