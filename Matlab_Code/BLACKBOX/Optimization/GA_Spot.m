function [PARA] = GA_Spot(x,BoomInfo,Num,varargin)
BoomInfo.Pianta.freccia=x(1)*pi/180/50;
BoomInfo.Pianta.l=x(2)/50;
BoomInfo.Profile.Chord=BoomInfo.Pianta.l/(x(3)/10);

[BoomInfo] = Boom3DShape(BoomInfo);

Chi=0.80;
D=pi/4;
theta=8*pi/180;
try
A = BoomSpotArea(BoomInfo,Num,D,theta,Chi,'plot');
PARA=1/A;
if A==0
    PARA=1000;
end
catch
    PARA=1000;
end

PARA
end

