function [dy]=EquationOfMotions(t,y)

% dy(1)=(M1-(I33-I22)y(2)y(3))/I11;
% dy(2)=(M2-(I11-I33)y(1)y(3))/I22;
% dy(3)=(M3-(I22-I11)y(1)y(2))/I33;
m=0.302; %Kg
I33=0.00502782489; %Kh m^2
R = 0.3048; %m
chord = 0.0508; %m hp
Irr=0.5*m*R*R;
Ms=0;
g=9.81;

geo.R = R;
geo.c = chord;
%from aero_BET

[L, Mn] = aero_BET(y, geo, 'False', 'VASS')

M3=0;
%y=[dtheta dphi dpsi theta phi psi dx dy dz x y z];
dy(1)=(Mn-(I33-Irr)*y(2)^2*sin(y(4))*cos(y(4))-I33*y(3)*y(2)*sin(y(4)))/Irr;
dy(2)=(Ms+I33*y(1)*(y(3)+y(2)*cos(y(4)))-2*Irr*y(1)*y(2)*cos(y(4)))/Irr/sin(y(4));
dy(3)=(M3-I33*dy(2)*cos(y(4))+I33*y(2)*y(1)*sin(y(4)))/I33;

dy(4)=y(1);
dy(5)=y(2);
dy(6)=y(3);

dy(7)=-L*sin(y(4))*sin(y(5))/m;
dy(8)=L*sin(y(4))*cos(y(5))/m;
dy(9)=(L*cos(y(4))-m*g)/m;

dy(10)=y(7);
dy(11)=y(8);
dy(12)=y(9);
dy=dy';
end