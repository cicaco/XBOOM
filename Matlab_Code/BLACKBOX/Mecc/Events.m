
function [value, isterminal, direction] = Events(T, Y)


value      = (Y(12) <= 0.01); % tocco con il terreno

if T>2 && value==0
    value = (sqrt(Y(10)^2+Y(11)^2+Y(12)^2)<=3); % distanza dal punto di lancio 
    %a cui interrompo l'integrazione: 3 metri
end

if T<1 && value==0
    value=( Y(2)>= 90); % controllo sull'inclinazione del boomerang 
end

isterminal = 1;   % Stop the integration
direction  = 0;
end
