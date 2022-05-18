clear all;
close all;
clc

%L lift orthogonal to the disk plane 
%Mn coefficient of momentum in respect to the n vector(roll moment)
%y is the state variable vector at the instant of time t
%geo geometry file -> first approximation -> it is just the radius

addpath(genpath('../'));
run Main_geometry.m

%Aerodynamics intialization of the boomerang aerodynamics Blade Element
%Theory (BET)
% as a first step it is implementedd the model described in the papershelp 
% (Boomerang flight dynamics Vassber) without drag contribution
% TO DO
% - geo generic shape
% - add effect of reverse flow and contribution of sweep -> with numerical
%   integrals
% - add drag

%% CHOOSE WHAT TO COMPUTE
plot_vn = false;
model   = "FULL";

%% COMPUTING THE AERODYNAMIC CENTER POSITION

%plot the boomerang plant and aerodynamics axis center
Plant3D = Profile3D(x_0,y_0,z_0,x_f,y_f,z_f,k_1,k_2,k_3,k_4,k_5,k_6,k_7,k_8);
Plant3D = Plant3D';


%number of points on the boundary
n_pt = length(Plant3D);
figure;
plot(Plant3D(1:n_pt/4,1),Plant3D(1:n_pt/4,2), 'b');
hold on
plot(Plant3D(n_pt/4+1:n_pt/2,1),Plant3D(n_pt/4+1:n_pt/2,2),'b');

grid on;
axis equal;

% LEL -> leading edge left, TEL -> trailing edge left
% they are sorted from the center to the tip -> that's why flip
LEL = Plant3D(1:n_pt/4,:);
TEL = flip(Plant3D(n_pt/4+1:n_pt/2,:));
LER = flip(Plant3D(n_pt/2+1:3/4*n_pt,:));
TER = Plant3D(3/4*n_pt+1:n_pt,:);

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



Rot  = [cos(pi) sin(pi);
     -sin(pi) cos(pi)];
X_l  = [Plant3D(n_pt/2+1:3/4*n_pt,1),Plant3D(n_pt/2+1:3/4*n_pt,2)]*Rot;
X_r  = [Plant3D(3/4*n_pt+1:n_pt,1), Plant3D(3/4*n_pt+1:n_pt,2)]*Rot;
ac_L = ac_L*Rot;
plot(ac_R(:,1), ac_R(:,2), 'r');
grid on;

% figure;
% %plot(Plant3D(n_pt/2+1:3/4*n_pt,1),Plant3D(n_pt/2+1:3/4*n_pt,2), 'b');
% plot(X_l(:,1),X_l(:,2),'b');
% hold on;
% %plot(Plant3D(3/4*n_pt+1:n_pt,1),Plant3D(3/4*n_pt+1:n_pt,2), 'b');
% plot(X_r(:,1),X_r(:,2), 'b');
% plot(ac_L(:,1), ac_L(:,2), 'm');
% title('Aerodynamics centers coordinates');
% axis equal;
% grid on;


%mean surface pf 1 blade
c_mean = sum(chord_L)/length(chord_L);
semispan = LEL(end,2);
S_ref = c_mean*semispan;
%% PLOT CHORD WISE DISTRIBUTION
figure;
plot(Plant3D(1:n_pt/4,2), chord_L ,'b')
grid on;
axis equal;

%% COMPUTING LOCAL QUANTITIES FROM FLIGHT DYNAMICS and geometry
R = max(Plant3D(1:n_pt/4,2));
S_ref = R*c_mean;
% V -> sqrt(Vx^2+Vy^2+Vz^2)
V = 10;
% chi -> (psi^(dot)*R)/V
chi = 1.2;
rho = 1.2;
% alpha = Vz/V
alpha = 2*pi/180;
%% 2D   AERODYNAMICS MODEL
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
%disp(plot_vn)
if plot_vn == true
    r     = linspace(0,R,100);
    psi = linspace(0,2*pi, 100); 
    [R_m,PSI_m] = meshgrid(r,psi);
    X = R_m.*cos(PSI_m);
    Y = R_m.*sin(PSI_m);
    contourf(X,Y, V_n(R_m, PSI_m));
    tit = strcat('Dimensionless Nomal Velocity V_N \chi = ', num2str(chi));
    title(tit)
    colorbar
end
%% VASSBER MODEL
if model == "VASS";
    chi_c = min(chi,1);
    Cl = Cl0*(0.5+(chi^2)/3)+ alpha*Cl_a*((chi/2+1/(4*chi))*(1-(2*sqrt(2)/pi)*...
         sqrt(1-chi_c))+(3/(2*pi))*sqrt(1-chi_c^2));
    Cm = Cl0*chi/3 + alpha*Cl_a*((1/4-1/(16*chi^2))*(1-(2*sqrt(2)/pi)*sqrt(1-chi_c))...
         +(sqrt(1-chi_c^2)/(4*pi))*((1/(2*chi_c))+chi_c));
end
%% Discrete integration
if model=='FULL';
    alpha_e = @(r,psi) alpha.*V./(abs(V_n(r,psi)));
    %local cl -> it must be integrated spanwise and over half period
    Cl_int = @(r,psi) ((V_n(r,psi)/V).^2).*Cl_2D(alpha_e(r,psi));
    % jacobian not considered as he does
    Cl     = (c_mean/(pi*R*c_mean))*integral2(Cl_int, 0, R, 0, pi);
    
    Cm_int = @(r,psi) ((V_n(r,psi)/V).^2).*Cl_2D(alpha_e(r,psi)).*(r.*cos(psi));
    % da rivedere!!
    Cm     = (c_mean/(pi*R^2*c_mean))*integral2(Cl_int, 0, R, 0, pi);
end

L  = 2*0.5*rho*V^2*S_ref*Cl;
Mn = 2*0.5*rho*V^2*S_ref*R*Cm;

%geo.x_vect = Plant3D(:,1);
%geo.y_vect = Plant3D(:,2);
% sorted for increasing value of y
geo.LEL = LEL;
geo.TEL = TEL;
geo.LER = LER;
geo.TER = TER;
geo.R      = R;
geo.c_mean = c_mean;
%saving data to directly recall it
save('2DPlant.mat', 'geo')