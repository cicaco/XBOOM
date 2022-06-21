
function [value, isterminal, direction] = Events(T, Y)
% EVENTS returns check and stop criteria for boomerang flight dynamics
% simulation (equation of motions)

% [value, isterminal, direction] = EVENTS(T, Y)
%
% given:
% - T -> instant of time
% - Y -> state vector [theta; phi; psi; p; q; r; ux; uy; uz; x; y; z]...
% ... everything in BODY framework apart from x, y, z  in inertial one
%
% returns:
% - value -> Boolean 
%       - z <= 0.01                              -> ground contact
%       - distance from starting position <= 3 m -> boomerang comes back
%       - phi >= 90 degree                       -> inclination check
% - isterminal ??
% - direction  ??
value      = (Y(12) <= 0.01); % tocco con il terreno

if T>2 && value==0
    value = (sqrt(Y(10)^2+Y(11)^2+Y(12)^2)<=1); % distanza dal punto di lancio 
    %a cui interrompo l'integrazione: 3 metri
end

if T<1 && value==0
    value=( Y(2)>= 90); % controllo sull'inclinazione del boomerang 
end

isterminal = 1;   % Stop the integration
direction  = 0;
end
