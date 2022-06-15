function [CM]=CM_naca0012(AoA)
%CM compute the CM for an angle of attack range -180/180
%CM=CM(AoA) computes the momentum coefficent for a given AoA
% INPUT: AoA (Angle of Attack in rad);
% OUTPUT: CM (Adimensional Momentum coefficient);
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
% %sorting CL
% CL_n      = data(:,2);
% 
% %% transport moment for cm at 3/4 ac
% CM_n    = data(:,5);
% CM_n    = CM_n + CL_n*0.5; %0.5 is half the chord
% CM_m    = -flip(data(:,5));
% CM_t    = [ CM_n; zeros(6,1); ...
%      pol.Cm; zeros(6,1); CM_m];
% interpolation
global alpha_t_CM 
global CM_t

alpha_eval=AoA*180/pi;
if alpha_eval<-180 || alpha_eval>180
    fprintf('error')
end
CM = interp1(alpha_t_CM, CM_t, alpha_eval);