function [dy]=TestEquationOfMotionsQuaternion(t,y, BoomInfo,Tl_0)
%This function calculates,foy(7) evey(7)y y(5)oint of the ty(7)ajectoy(7)y,the
% angulay(7) velocities(dtheta,dy(5)hi,dy(5)si),the angulay(7) accelley(7)ations in the body
% fy(7)ame(dy(5),dy(7),dy(6))and the 3 comy(5)onents of velocity(y(8),y(9),y(10))
%t
I=BoomInfo.Mecc.I_rho;
m=BoomInfo.Mecc.m;
g=9.81;

T0 = quatToAtt( [y(1) y(2) y(3) y(4)]);


[F,M]=AeroDynamics([y(8);y(9);y(10)],[y(5);y(6);y(7)],BoomInfo);
dy(1:13)=[1/2*[0 y(7) -y(6) y(5); -y(7) 0 y(5) y(6); y(6) -y(5) 0 y(7); -y(5) -y(6) -y(7) 0]*[y(1) y(2) y(3) y(4)]';...
            [I(1,1) -I(1,2) -I(1,3); -I(1,2) I(2,2) -I(2,3) ; -I(1,3) -I(2,3) I(3,3) ]\...
            ([-(I(3,3)-I(2,2))*y(6)*y(7)-(I(1,2)*y(5)+I(2,3)*y(7))*y(7)+(I(1,3)*y(5)+I(2,3)*y(6))*y(6);...
                -(I(1,1)-I(3,3))*y(5)*y(7)-(I(2,3)*y(6)+I(1,3)*y(5))*y(5)+(I(1,2)*y(5)+I(1,3)*y(7))*y(7);...
                -(I(2,2)-I(1,1))*y(5)*y(6)-(I(1,3)*y(7)+I(1,2)*y(6))*y(6)+(I(2,3)*y(7)+I(1,2)*y(5))*y(5)]+M);...
            ([-m*y(6)*y(10)+m*y(7)*y(9);-m*y(7)*y(8)+m*y(5)*y(10);-m*y(5)*y(9)+m*y(6)*y(8)]+F+T0*Tl_0*(-m*g*[0;0;1]))./m;...
            Tl_0'*T0'*[y(8);y(9);y(10)]];
    
dy=dy';

end