function [CD]=CD(AoA)
% CD_NEW compute the CD for an AoA range -pi/pi degree
%
% CD = CD_NEW(AoA) computes the drag coefficient for a given AoA
%
% INPUT: AoA (Angle of Attack in rad);
% OUTPUT: CD (Adimensional drag coefficient);
%
% Taken from: Azuma, Akira, et al. "Flight dynamics of the boomerang, part 1: 
% fundamental analysis." Journal of guidance, control, and dynamics 
% 27.4 (2004): 545-554.

alpha=AoA*180/pi;
if alpha<-180 || alpha>180
    fprintf('error')
elseif -180<=alpha && alpha<-170
    CD=0.01;
elseif -170<=alpha && alpha<-160
    CD= 0.4/10*(alpha+170)+0.01;%ok
elseif -160<=alpha && alpha<-130
    CD= 0.84/30*(alpha+160)+0.41;%ok
elseif -130<=alpha && alpha<-90
    CD= 0.45/40*(alpha+130)+1.25;%ok
elseif -90<=alpha && alpha<-50
    CD= -0.45/40*(alpha + 90) + 1.7;%ok
elseif -50<=alpha && alpha<-20
    CD= -0.84/30*(alpha+50)+1.25;%ok
elseif -20<=alpha && alpha<-10
    CD= -0.4/10*(alpha+20)+0.41;%ok
elseif -10<=alpha && alpha<10
    CD = 0.01;%ok
elseif 10<=alpha && alpha<20
    CD = 0.4/10*(alpha-10)+0.01;%ok
elseif 20<=alpha && alpha<50
    CD = 0.84/30*(alpha-20) +0.41;%ok
elseif 50<=alpha && alpha<90
    CD = 0.45/40*(alpha-50)+1.25;%ok
elseif 90<=alpha && alpha<130
    CD = -0.45/40*(alpha - 90) + 1.7;%ok
elseif 130<=alpha && alpha<160
    CD= -0.84/30*(alpha-130)+1.25;%ok
elseif 160<=alpha && alpha<170
    CD= -0.4/10*(alpha-160)+0.41;%ok
elseif 170<=alpha && alpha<=180
    CD = 0.01;%ok


            
end
end
