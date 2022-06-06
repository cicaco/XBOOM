function [F,M]=aero(u,omega)
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
    -cos(sigma)*cos(coning), sin(sigma)*cos(coning), +sin(coning)
    sin(sigma)*sin(pitch)-cos(sigma)*sin(coning)*cos(pitch), -cos(sigma)*sin(pitch)-sin(sigma)*sin(coning)*cos(pitch), cos(coning)*cos(pitch)];

% Tj'*Tj
%spanwise wing distance
eta=linspace(0,R,10);

%blade element position in body frame
rA=[];
for i=1:length(eta)
rA=[rA, [xac;0;0]+Tj'*[0;eta(i);0]];
end

%velocity of blade element
v=[];
for i=1:length(eta)
v=[v, u+cross(omega,rA(:,i))];
end

v_ind=0; %ipotesi iniziale, DA CAMBIARE

%relative velocity of blade in blade frame
w=[];
for i=1:length(eta)
w=[w, Tj*(-v(:,i)-[0;0;v_ind])];
end

%AoA
AoA=[];
%svergolamento
twist=zeros(1,length(eta));
for i=1:length(eta)
AoA=[AoA, atan2(w(3,i),w(1,i))+twist(i)];
end

%BET
FA=[];
MA=[];
for i=2:length(eta)
FA=[FA, (eta(i)-eta(i-1))*0.5*1.225*c*norm(w(:,i))^2*[-CL(AoA(i))*sin(AoA(i))+CD(AoA(i))*cos(AoA(i)); 0; CL(AoA(i))*cos(AoA(i))+CD(AoA(i))*sin(AoA(i))]];
MA=[MA,(eta(i)-eta(i-1))*0.5*1.225*c*norm(w(:,i))^2*[(CL(AoA(i))*cos(AoA(i))+CD(AoA(i))*sin(AoA(i)))*eta(i); c*CM(AoA(i)); (CL(AoA(i))*sin(AoA(i))-CD(AoA(i))*cos(AoA(i)))*eta(i)] ];
end

%forze su sdr body
F=[];
M=[];
for i=1:length(FA)
    F=[F, (Tj')*FA(:,i)];
    M=[M, (Tj')*MA(:,i)+cross(rA(:,i+1),(Tj')*FA(:,i))];
end

F=sum(F,2);
M=sum(M,2);

%SECONDA PALA

%sdr PALA j
sigma=(240)*pi/180;
coning=0; %tipo diedro (rot asse x2)
pitch=0; %pitch della pala (rot asse y3)

%matrice di rotazione da body a blade
Tj=[sin(sigma)*cos(pitch)+cos(sigma)*sin(coning)*sin(pitch), -cos(sigma)*cos(pitch)+sin(sigma)*sin(coning)*sin(pitch), -cos(coning)*sin(pitch)
    -cos(sigma)*cos(coning), sin(sigma)*cos(coning), +sin(coning)
    sin(sigma)*sin(pitch)-cos(sigma)*sin(coning)*cos(pitch), -cos(sigma)*sin(pitch)-sin(sigma)*sin(coning)*cos(pitch), cos(coning)*cos(pitch)];

%spanwise wing distance
eta=linspace(0,R,10);

%blade element position in body frame
rA=[];
for i=1:length(eta)
rA=[rA, [xac;0;0]+Tj'*[0;eta(i);0]];
end

%velocity of blade element
v=[];
for i=1:length(eta)
v=[v, u+cross(omega,rA(:,i))];
end

v_ind=0; %ipotesi iniziale, DA CAMBIARE

%relative velocity of blade in blade frame
w=[];
for i=1:length(eta)
w=[w, Tj*(-v(:,i)-v_ind)];
end

%AoA
AoA=[];
%svergolamento
twist=zeros(1,length(eta));
for i=1:length(eta)
AoA=[AoA, atan2(w(3,i),w(1,i))+twist(i)];
end

%BET
FA=[];
MA=[];
for i=2:length(eta)
FA=[FA, (eta(i)-eta(i-1))*0.5*1.225*c*norm(w(:,i))^2*[-CL(AoA(i))*sin(AoA(i))+CD(AoA(i))*cos(AoA(i)); 0; CL(AoA(i))*cos(AoA(i))+CD(AoA(i))*sin(AoA(i))]];
MA=[MA,(eta(i)-eta(i-1))*0.5*1.225*c*norm(w(:,i))^2*[(CL(AoA(i))*cos(AoA(i))+CD(AoA(i))*sin(AoA(i)))*eta(i); c*CM(AoA(i)); (CL(AoA(i))*sin(AoA(i))-CD(AoA(i))*cos(AoA(i)))*eta(i)] ];
end

%forze su sdr body
F2=[];
M2=[];
for i=1:length(FA)
    F2=[F, Tj'*FA(:,i)];
    M2=[M, Tj'*MA(:,i)+cross(rA(:,i+1),Tj'*FA(:,i))];
end
F2=sum(F2,2);
M2=sum(M2,2);

F=F+F2;
M=M+M2;
