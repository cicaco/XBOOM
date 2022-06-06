clear all
% clc

%massa pala
m=0.130/2; %Kg
c=0.0488; %m

%sdr PALA j
sigma=120*pi/180;
coning=0; %tipo diedro (rot asse x2)
pitch=0; %pitch della pala (rot asse y3)

%matrice di rotazione da body a blade
Tj=[sin(sigma)*cos(pitch)+cos(sigma)*sin(coning)*sin(pitch), -cos(sigma)*cos(pitch)+sin(sigma)*sin(coning)*sin(pitch), -cos(coning)*sin(pitch)
    cos(sigma)*cos(coning), sin(sigma)*cos(coning), +sin(coning)
    sin(sigma)*sin(pitch)-cos(sigma)*sin(coning)*cos(pitch), -cos(sigma)*sin(pitch)-sin(sigma)*sin(coning)*cos(pitch), cos(coning)*cos(pitch)];
xac=0.0723; %va cambiato tra prima e seconda pala (DA FARE)
xcg=0.145;
Xcg1_bodyframe=-([xac; 0; 0]+Tj'*[0; xcg;0]);

%inerzie in blade frame
I_xsi=1.9*10^-3;
I_eta=4.88*10^-6;
I_zeta=1.90*10^-3;
I_xsi_eta=5.14*10^-21;

Iblade=[I_xsi I_xsi_eta 0
    I_xsi_eta I_eta 0
    0 0 I_zeta];
% 
I_blade_body=Tj'*Iblade*Tj;

% I_blade_body=Tj*Iblade*Tj';

J1=[];
for i=1:3
    for j=1:3
        J1(i,j)=I_blade_body(i,j)+m*((norm(Xcg1_bodyframe))^2*isequal(i,j)-Xcg1_bodyframe(i)*Xcg1_bodyframe(j));
    end
end

%% seconda pala
%massa pala
m=0.130/2; %Kg

%sdr PALA j
sigma=240*pi/180;
coning=0; %tipo diedro (rot asse x2)
pitch=0; %pitch della pala (rot asse y3)

%matrice di rotazione da body a blade
Tj=[sin(sigma)*cos(pitch)+cos(sigma)*sin(coning)*sin(pitch), -cos(sigma)*cos(pitch)+sin(sigma)*sin(coning)*sin(pitch), -cos(coning)*sin(pitch)
    cos(sigma)*cos(coning), sin(sigma)*cos(coning), +sin(coning)
    sin(sigma)*sin(pitch)-cos(sigma)*sin(coning)*cos(pitch), -cos(sigma)*sin(pitch)-sin(sigma)*sin(coning)*cos(pitch), cos(coning)*cos(pitch)];

%caratteristiche pala
xac=0.0723+c/2;
xcg=0.145;
%cg pala in body frame
Xcg1_bodyframe=-([xac; 0; 0]+Tj'*[0; xcg;0]);

%inerzie in blade frame
I_xsi=1.9*10^-3;
I_eta=4.88*10^-6;
I_zeta=1.90*10^-3;
I_xsi_eta=5.14*10^-21;

Iblade=[I_xsi I_xsi_eta 0
    I_xsi_eta I_eta 0
    0 0 I_zeta];

I_blade_body=Tj'*Iblade*Tj;
% I_blade_body=Tj*Iblade*Tj';
J2=[];
for i=1:3
    for j=1:3
        J2(i,j)=I_blade_body(i,j)+m*((norm(Xcg1_bodyframe))^2*isequal(i,j)-Xcg1_bodyframe(i)*Xcg1_bodyframe(j));
    end
end

%inerzia boomerang
J=J1+J2;
