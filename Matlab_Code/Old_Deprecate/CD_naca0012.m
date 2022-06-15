function [CD]=CD_naca0012(AoA)
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
% %sorting alpha
% alpha   = data(:,1);
% alpha_m = -flip(data(:,1));
% alpha_t = [ alpha-180; linspace(-160,-40,6)'; ...
%     pol.alpha; linspace(40, 160, 6)'; alpha_m+180] ;
% 
% %sorting CD
% CD_n      = data(:,3);
% CD_m      = flip(data(:,3));
% CD_t    = [  CD_n; 1.2*ones(6,1); ...
%      pol.CD; 1.2*ones(6,1); CD_m];
 
% interpolation
global alpha_t_CD 
global CD_t
alpha_eval=AoA*180/pi;
if alpha_eval<-180 || alpha_eval>180
    fprintf('error')
end

CD = interp1(alpha_t_CD, CD_t, alpha_eval);