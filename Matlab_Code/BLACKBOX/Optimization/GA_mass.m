function [PARA] = GA_mass(x,lb,ub,BoomInfo,D,theta,Chi)
if x>=lb && x<=ub 
BoomInfo.Mecc.I_rho=BoomInfo.Mecc.I_rho./BoomInfo.Mecc.Dens;
BoomInfo.Mecc.Dens=x;
BoomInfo.Mecc.m=BoomInfo.Mecc.V*x;
BoomInfo.Mecc.I_rho=BoomInfo.Mecc.I_rho.*x;
try
PARA = - SpotArea(BoomInfo,D,theta,Chi);
catch
    PARA=0;
end
else
    PARA=0;
end

