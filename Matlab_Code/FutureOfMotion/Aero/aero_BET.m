function [L, Mn] = aero_BET(y, geo, plot_vn, model)
% AERODYNAMIC FORCES COMPUTEDD FROM THE
%L lift orthogonal to the disk plane 
%Mn coefficient of momentum in respect to the n vector(roll moment)
%y is the state variable vector at the instant of time t
%geo geometry file -> first approximation -> it is just the radius

addpath(genpath('Aero'));

%Aerodynamics intialization of the boomerang aerodynamics Blade Element
%Theory (BET)
% as a first step it is implementedd the model described in the papers
% (Boomerang flight dynamics Vassber) without drag contribution
% TO DO
% - geo generic shape
% - add effect of reverse flow and contribution of sweep -> with numerical
%   integrals
% - add drag
T_ib=[cos(y(5))*cos(y(6))-sin(y(5))*cos(y(4))*sin(y(6)) sin(y(5))*cos(y(6))+cos(y(5))*cos(y(4))*sin(y(6)) sin(y(4))*sin(y(6))
    -cos(y(5))*sin(y(6))-sin(y(5))*cos(y(4))*cos(y(6)) -sin(y(5))*sin(y(6))+cos(y(5))*cos(y(4))*cos(y(6)) sin(y(4))*cos(y(6))
    sin(y(5))*sin(y(4)) -cos(y(5))*sin(y(4)) cos(y(4))];
V_vect=(T_ib)*[y(7); y(8); y(9)];
%% extracting geometric quantity
R     = geo.R;
chord = geo.c;
S_ref = R*chord;
rho   = 1.2; %kg/m^3
%% COMPUTING LOCAL QUANTITIES FROM FLIGHT DYNAMICS
% V -> sqrt(Vx^2+Vy^2+Vz^2)
V = sqrt(y(7)^2+y(8)^2+y(9)^2);
% V = sqrt(V_vect(1)^2+V_vect(2)^2+V_vect(3)^2);
% chi -> (psi^(dot)*R)/V
chi = y(3)*R/V;
% alpha = Vz/V
% alpha = -V_vect(3)/V;
alpha=-y(9)/V;
alpha*180/pi;

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
% if plot_vn == 'True'
%     r     = linspace(0,R,100);
%     psi = linspace(0,2*pi, 100); 
%     [R_m,PSI_m] = meshgrid(r,psi);
%     X = R_m.*cos(PSI_m);
%     Y = R_m.*sin(PSI_m);
%     contourf(X,Y, V_n(R_m, PSI_m));
%     tit = strcat('Dimensionless Nomal Velocity V_N \chi = ', num2str(chi));
%     title(tit)
%     colorbar
% end
%% VASSBER MODEL
if model == "VASS";
    chi_c = min(chi,1);
    Cl = Cl0*(0.5+(chi^2)/3)+ alpha*Cl_a*((chi/2+1/(4*chi))*(1-(2*sqrt(2)/pi)*...
         sqrt(1-chi_c))+(3/(2*pi))*sqrt(1-chi_c^2));
    Cm = Cl0*chi/3 + alpha*Cl_a*((1/4-1/(16*chi^2))*(1-(2*sqrt(2)/pi)*sqrt(1-chi_c))...
         +(sqrt(1-chi_c^2)/(4*pi))*((1/(2*chi_c))+chi_c));
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

% if model == "FULL";
% %plot the boomerang plant and aerodynamics axis center
% Plant3D=Profile3D(x_0,y_0,z_0,x_f,y_f,z_f,k_1,k_2,k_3,k_4,k_5,k_6,k_7,k_8);
% Plant3D = Plant3D';
% %number of points on the boundary
% n_pt = length(Plant3D);
% figure;
% plot(Plant3D(1:n_pt/4,1),Plant3D(1:n_pt/4,2), 'b');
% hold on
% plot(Plant3D(n_pt/4+1:n_pt/2,1),Plant3D(n_pt/4+1:n_pt/2,2),'b');
% 
% grid on;
% axis equal;
% 
% % LEL -> leading edge left, TEL -> trailing edge left
% % they are sorted from the center to the tip -> that's why flip
% LEL = Plant3D(1:n_pt/4,:);
% TEL = flip(Plant3D(n_pt/4+1:n_pt/2,:));
% LER = Plant3D(n_pt/2+1:3/4*n_pt,:);
% TER = flip(Plant3D(3/4*n_pt+1:n_pt,:));
% 
% % chord distribution in x direction
% chord_L = abs(LEL(:,1)-TEL(:,1));
% chord_R = abs(LER(:,1)-TER(:,1));
% 
% %aerodynamics center init
% ac_R = LEL;
% ac_L = TER;
% % aerodynamics centers computation
% % IT MUST BE CORRECTED -> BECAUSE WHEN THE FLOW ENTER FROM TRAILING EDGE
% % THE AERODYNAMIC CENTER SWITCH POSITION
% 
% ac_R(:,1) = LEL(:,1) + 0.25*chord_L(:,1);
% ac_L(:,1) = LER(:,1) - 0.25*chord_R(:,1);
% 
% 
% 
% Rot  = [cos(pi) sin(pi);
%      -sin(pi) cos(pi)]
% X_l  = [Plant3D(n_pt/2+1:3/4*n_pt,1),Plant3D(n_pt/2+1:3/4*n_pt,2)]*Rot
% X_r  = [Plant3D(3/4*n_pt+1:n_pt,1), Plant3D(3/4*n_pt+1:n_pt,2)]*Rot
% ac_L = ac_L*Rot;
% plot(ac_R(:,1), ac_R(:,2), 'r');
% grid on;
% 
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
% 
% 
% %mean surface pf 1 blade
% c_mean = sum(chord_L)/length(chord_L);
% semispan = LEL(end,2);
% S_ref = c_mean*semispan;
% 
% %% Discrete integration
% 
%     alpha_e = @(r,psi) alpha.*V./(abs(V_n(r,psi)));
%     %local cl -> it must be integrated spanwise and over half period
%     Cl_int = @(r,psi) ((V_n(r,psi)/V).^2).*Cl_2D(alpha_e(r,psi));
%     % jacobian not considered as he does
%     Cl     = (c_mean/(pi*R*c_mean))*integral2(Cl_int, 0, R, 0, pi);
%     
%     Cm_int = @(r,psi) ((V_n(r,psi)/V).^2).*Cl_2D(alpha_e(r,psi)).*(r.*cos(psi));
%     % da rivedere!!
%     Cm     = (c_mean/(pi*R*c_mean^2))*integral2(Cl_int, 0, R, 0, pi);
% end
L  = 2*0.5*rho*V^2*S_ref*Cl;
Mn = 2*0.5*rho*V^2*S_ref*R*Cm;
end
