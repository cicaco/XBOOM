function [PARA] = GA_Spot(x,BoomInfo,D,theta,Chi)
BoomInfo.Pianta.freccia=x(1)*pi/180;
BoomInfo.Pianta.l=x(2);
BoomInfo.Profile.Chord=BoomInfo.Pianta.l/(x(3));

[BoomInfo] = Boom3DShape(BoomInfo);

try
PARA = - SpotArea(BoomInfo,D,theta,Chi);
catch
    PARA=0;
end

