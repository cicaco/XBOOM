function [CL]=CL_naca0012(AoA)
%CL compute the CL for an angle of attack range -180/180
%CL=CL(AoA) computes the lift coefficent for a given AoA
% INPUT: AoA (Angle of Attack in rad);
% OUTPUT: CL (Adimensional Lift coefficient);
% Computed from XFOIL NACA 0012 coefficients

%load computed quantity from XFoil
% data = load('reverse_0012_coeff');
% load norm_polar
% 
% 
% 
% %sorting alpha
% alpha   = data(:,1);
% alpha_m = -flip(data(:,1));
% alpha_t = [ alpha-180; linspace(-160,-40,6)'; ...
%     pol.alpha; linspace(40, 160, 6)'; alpha_m+180] ;
% 
% %sorting CL
% CL_n      = data(:,2);
% CL_m    = -flip(data(:,2));
% CL_t    = [ CL_n; zeros(6,1); ...
%      pol.CL; zeros(6,1); CL_m] ;
% interpolation
global alpha_t_CL 
global CL_t
alpha_eval=AoA*180/pi;
if alpha_eval<-180 || alpha_eval>180
    fprintf('error')
end
CL = interp1(alpha_t_CL, CL_t, alpha_eval);