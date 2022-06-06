function Plant3D=Profile3D(x_0,y_0,z_0,x_f,y_f,z_f,k_1,k_2,k_3,k_4,k_5,k_6,k_7,k_8)
%PROFILE3D Summary of this function goes here
%   We have 8 parameters that corresponds to 8 point in the plane of the
%   boomerang ( If you want to understand this parameters see the figure on
%   the report
Origin=[x_0 y_0 z_0]; %FIX
P1=[ x_0 k_1 z_0]; %k1 parameter that can be choiche
P2=[ k_2 k_3 z_0];
P3=[ k_4 k_5 z_0];
P4=[k_6 k_5 z_0];
P5=[k_7 k_5 z_0];
P6=[k_8 k_3 z_0];
P7=[x_f k_1 z_f];
Final=[x_f y_f z_f];
%Example:
P=[Origin' P1' P2' P3' P4'];
t=linspace(0,1,100);
Q3D_lead=Bezier(P,t);


% figure()
% plot(Q3D_lead(1,:),Q3D_lead(2,:),'b','LineWidth',2),
% hold on
% plot(Q3D_lead(1,:),-Q3D_lead(2,:),'b','LineWidth',2),
% plot(P(1,:),P(2,:),'g:','LineWidth',2) % plot control polygon
% plot(P(1,:),P(2,:),'ro','LineWidth',2) % plot control points
P=  [P4' P5' P6' P7' Final'];
t=linspace(0,1,100);
Q3D_trailing=Bezier(P,t);
Plant3D=[Q3D_lead(1,:) Q3D_trailing(1,:)   fliplr(Q3D_trailing(1,:)) fliplr(Q3D_lead(1,:)); Q3D_lead(2,:)  Q3D_trailing(2,:) fliplr(-Q3D_trailing(2,:)) fliplr(-Q3D_lead(2,:)) ];


% plot(Q3D_trailing(1,:),Q3D_trailing(2,:),'b','LineWidth',2),
% hold on
% plot(Q3D_trailing(1,:),-Q3D_trailing(2,:),'b','LineWidth',2),
% plot(P(1,:),P(2,:),'g:','LineWidth',2) % plot control polygon
% plot(P(1,:),P(2,:),'ro','LineWidth',2) % plot control points
% axis equal
% 
% grid on
end

