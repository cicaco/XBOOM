clear all;
close all;
clc

run Main_geometry

addpath(genpath('Aero'));
%Aerodynamics intialization of the boomerang aerodynamics Blade Element
%Theory (BET)4
% as a first step it is implementedd the model described in the papers
% (Boomerang flight dynamics Vassber) without drag contribution
% TO DO
% - add effect of reverse flow and contribution of sweep -> with numerical
%   integrals
% - add drag

% the ration between translational and angular velodity is given by the
% flight dynamics of the boomerang
chi = 0.85;
R   = 8;
Cl0 = 0;
Cl_a = 2*pi;
alpha  = 2*pi/180; % 

%aerodyncamics coefficient for direct and reversal airflow
Cl0m = 0;
Cl0p = 1;

%% PLOT THE DIRECTION OF NORMAL VELOCITY
% more than 0 enters from the leading edge, less than 0 trailing edge
%non dimensional normal velocity
%r no dimensional radial coordinate
V_nt = @(r, psi) cos(psi) + chi*r; % -> change it, it should be a matrix
model = "VASS"
r     = linspace(0,1,100)
theta = linspace(0,2*pi, 100) 
polarcont(r,theta,V_nt(r, theta))
%% VASSBER MODEL
if model == "VASS";
chi_c = min(chi,1);
    Cl = Cl0*(0.5+chi^2/3)+ alpha*Cl_a*((chi/2+1/(4*chi)*(1-2*sqrt(2)/pi*...
         sqrt(1-chi_c))+3/(2*pi)*sqrt(1-chi_c^2)));
    Cm = Cl0*chi/3 + alpha*Cl_a*((1/4-1/(16*chi^2))*(1-2*sqrt(2)/pi*sqrt(1-chi_c))...
         +sqrt(1-chi_c^2)/(4*pi)*(1/(2*chi_c)+chi_c))  
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
% aerodynamics centers comp
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