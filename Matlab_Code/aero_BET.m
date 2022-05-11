clear all;
close all;
clc

run Main_geometry
close all;
addpath(genpath('Aero'));
%Aerodynamics intialization of the boomerang aerodynamics Blade Element
%Theory (BET)4
% as a first step it is implementedd the model described in the papers
% (Boomerang flight dynamics Vassber) without drag contribution
% TO DO
% - add effect of reverse flow and contribution of sweep -> with numerical
%   integrals
% - add drag

%initialize aerodynamics model
model = "FULL";

% the ration between translational and angular velodity is given by the
% flight dynamics of the boomerang
chi = 0.7;
alpha  = 2*pi/180; % 2 degree alpha
V = 10; % diciamo che ha una velocità di 10 m/s
%given by design
R   = 8;
%Quantity for the aerodynamic computation using Vassber model
Cl0 = 0;
Cl_a = 2*pi;
%cl_alpha and polar plot
Cl_2D  = @(alpha) 2*pi.*alpha; % considering a cl_alpha sunction of alpha
cm0 = 0; %sym profile

%% PLOT THE DIRECTION OF NORMAL VELOCITY
% more than 0 enters from the leading edge, less than 0 trailing edge
%non dimensional normal velocity
%r no dimensional radial coordinate
V_n = @(r, psi) V*(cos(psi) + chi.*r/R); % -> change it, it should be a matrix
r     = linspace(0,R,100);
psi = linspace(0,2*pi, 100); 
[R_m,PSI_m] = meshgrid(r,psi);
X = R_m.*cos(PSI_m);
Y = R_m.*sin(PSI_m);

contourf(X,Y, V_n(R_m, PSI_m));
tit = strcat('Dimensionless Nomal Velocity V_N \chi = ', num2str(chi));
title(tit)
colorbar
%polarcont(r,theta,V_nt(R, THETA))
%% VASSBER MODEL
if model == "VASS";
chi_c = min(chi,1);
    Cl = Cl0*(0.5+chi^2/3)+ alpha*Cl_a*((chi/2+1/(4*chi)*(1-2*sqrt(2)/pi*...
         sqrt(1-chi_c))+3/(2*pi)*sqrt(1-chi_c^2)));
    Cm = Cl0*chi/3 + alpha*Cl_a*((1/4-1/(16*chi^2))*(1-2*sqrt(2)/pi*sqrt(1-chi_c))...
         +sqrt(1-chi_c^2)/(4*pi)*(1/(2*chi_c)+chi_c));  
end
%% INVERSE AIRFLOW -> useless if in the previous model we change
%% cl and clalpha based on the sign of V_n
% elseif  model == "air_rev";
%     %not completed because this model desn't take into account something in
%     %the VASSBER model 
%     %it should work just fot chi more than 1
%     Cl0_b = (0.5+(chi^2)/3) + (1-Cl0m/Cl0p)*(1/(2*pi*chi)*(sqrt(1-chi^2)-(...
%         1-chi^2)^1.5 - chi*acos(chi)))
% end
%% COMPUTING THE AERODYNAMIC CENTER POSITION
% plot the boomerang plant and aerodynamics axis center
Plant3D=Profile3D(x_0,y_0,z_0,x_f,y_f,z_f,k_1,k_2,k_3,k_4,k_5,k_6,k_7,k_8);
Plant3D = Plant3D';
%number of points on the boundary
n_pt = length(Plant3D);
figure;
plot(Plant3D(1:n_pt/4,1),Plant3D(1:n_pt/4,2), 'b');
hold on
plot(Plant3D(n_pt/4+1:n_pt/2,1),Plant3D(n_pt/4+1:n_pt/2,2),'b');
plot(Plant3D(n_pt/2+1:3/4*n_pt,1),Plant3D(n_pt/2+1:3/4*n_pt,2), 'b');
plot(Plant3D(3/4*n_pt+1:n_pt,1),Plant3D(3/4*n_pt+1:n_pt,2), 'b');
grid on;

% LEL -> leading edge left, TEL -> trailing edge left
% they are sorted from the center to the tip -> that's why flip
LEL = Plant3D(1:n_pt/4,:);
TEL = flip(Plant3D(n_pt/4+1:n_pt/2,:));
LER = Plant3D(n_pt/2+1:3/4*n_pt,:);
TER = flip(Plant3D(3/4*n_pt+1:n_pt,:));

% chord distribution in x direction
chord_L = abs(LEL(:,1)-TEL(:,1));
chord_R = abs(LER(:,1)-TER(:,1));

%aerodynamics center init
ac_R = LEL;
ac_L = TER;
% aerodynamics centers computation
% IT MUST BE CORRECTED -> BECAUSE WHEN THE FLOW ENTER FROM TRAILING EDGE
% THE AERODYNAMIC CENTER SWITCH POSITION

ac_R(:,1) = LEL(:,1) + 0.25*chord_L(:,1);
ac_L(:,1) = LER(:,1) - 0.25*chord_R(:,1);


plot(ac_R(:,1), ac_R(:,2), 'r');
plot(ac_L(:,1), ac_L(:,2), 'm');
title('Aerodynamics centers coordinates');
axis equal;

%mean surface pf 1 blade
c_mean = sum(chord_L)/length(chord_L);
semispan = LEL(end,2);
S_ref = c_mean*semispan;

%% Discrete integration
if model == "FULL";
    alpha_e = @(r,psi) alpha.*V./(abs(V_n(r,psi)));
    %local cl -> it must be integrated spanwise and over half period
    Cl_int = @(r,psi) ((V_n(r,psi)/V).^2).*Cl_2D(alpha_e(r,psi));
    % jacobian not considered as he does
    Cl     = (c_mean/(pi*R*c_mean))*integral2(Cl_int, 0, R, 0, pi);
end

