function [L, Mn] = aero_BET(y, geo, model)
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

%% extracting geometric quantity
R     = geo.R;
chord = geo.c_mean;
S_ref = R*chord;
rho   = 1.2; %kg/m^3


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

%% VASSBER MODEL
if model == "VASS";
    % COMPUTING LOCAL QUANTITIES FROM FLIGHT DYNAMICS
    % V -> sqrt(Vx^2+Vy^2+Vz^2)
    V = sqrt(y(7)^2+y(8)^2+y(9)^2);
    % V = sqrt(V_vect(1)^2+V_vect(2)^2+V_vect(3)^2);
    % chi -> (psi^(dot)*R)/V
    chi = y(3)*R/V;
    % alpha = Vz/V
    %alpha = -V_vect(3)/V;
    alpha=-y(9)/V;
    alpha*180/pi
    
    chi_c = min(chi,1);
    Cl = Cl0*(0.5+(chi^2)/3)+ alpha*Cl_a*((chi/2+1/(4*chi))*(1-(2*sqrt(2)/pi)*...
         sqrt(1-chi_c))+(3/(2*pi))*sqrt(1-chi_c^2));
    Cm = Cl0*chi/3 + alpha*Cl_a*((1/4-1/(16*chi^2))*(1-(2*sqrt(2)/pi)*sqrt(1-chi_c))...
         +(sqrt(1-chi_c^2)/(4*pi))*((1/(2*chi_c))+chi_c));
    L  = 2*0.5*rho*V^2*S_ref*Cl;
    Mn = 0.5*rho*V^2*S_ref*R*Cm;
end

if model == "FULL";
    %this is a NOT averaged model over the period
    %everything is computed in body axes
    
    T_ib=[cos(y(5))*cos(y(6))-sin(y(5))*cos(y(4))*sin(y(6)) sin(y(5))*cos(y(6))+cos(y(5))*cos(y(4))*sin(y(6)) sin(y(4))*sin(y(6))
    -cos(y(5))*sin(y(6))-sin(y(5))*cos(y(4))*cos(y(6)) -sin(y(5))*sin(y(6))+cos(y(5))*cos(y(4))*cos(y(6)) sin(y(4))*cos(y(6))
    sin(y(5))*sin(y(4)) -cos(y(5))*sin(y(4)) cos(y(4))];
    V_vect=-(T_ib)*[y(7); y(8); y(9)];
    %alpha just to check the angle
    alpha = V_vect(3)/V_vect(1)*180/pi;
    %leading edge left
    LEL = geo.LEL;
    %trailing edge left
    TEL = geo.TEL;
    %leading edge left
    LER = geo.LER;
    %trailing edge right
    TER = geo.TER;
    %put together left and right head side
    LE = [LER; LEL];
    TE = [TER; TEL];
    y_i = LE(:,2);
    c_i = abs(LE(:,1)-TE(:,2));
    %velocity oh the i-th section
    Vx_i    = V_vect(1) + y(3).*y_i;
    V_i     = sqrt(V_vect(3)^2+Vx_i.^2);
    alpha_i = atan2(V_vect(3),Vx_i);
    cl_i    = zeros(length(y_i),1);
    %evaluating spanwise cl
    for ii = 1:length(y_i);
        %something like stall
        if alpha_i*180/pi<15;
            cl_i(ii) = Cl0 + Cl_a*alpha_i(ii);
        else
            cl_i = 0;
        end
    end
    L_i = 0.5*rho.*(V_i.^2).*c_i.*cl_i.*cos(alpha_i);
    M_i = L_i.*y_i;
    %discrete integration -> :'(
    L  = trapz(y_i, L_i)
    Mn = trapz(y_i, M_i)
    V_vect(1)
   
end
