function [PARA] = GA_Spot(x,BoomInfo,D,theta,Chi)
BoomInfo.Pianta.freccia=x(1)*pi/180/10;
BoomInfo.Pianta.l=x(2)/1000;
BoomInfo.Profile.Chord=BoomInfo.Pianta.l/(x(3)/100);

[BoomInfo] = Boom3DShape(BoomInfo);

try
PARA = - SpotArea(BoomInfo,D,theta,Chi);
catch
    PARA=0;
end

