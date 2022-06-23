%Initial Condition
function InitialConditionPlot(theta0,phi0,psi0,U,Om,BoomInfo)
% U,Om sistema body
num=BoomInfo.Geom3D.num;
p_c=BoomInfo.Geom3D.p_c;
CG=BoomInfo.Mecc.CG;
l=BoomInfo.Pianta.l;
% theta0=5*pi/180;
% phi0=45*pi/180;  % initial inclination of the boomerang -70 / -90 degrees; baseline -85 degrees
% psi0=160*pi/180;
Tl_0=[cos(theta0)*cos(psi0), cos(theta0)*sin(psi0), -sin(theta0)
    -cos(phi0)*sin(psi0)+sin(phi0)*sin(theta0)*cos(psi0), cos(phi0)*cos(psi0)+sin(phi0)*sin(theta0)*sin(psi0), sin(phi0)*cos(theta0)
    sin(phi0)*sin(psi0)+cos(phi0)*sin(theta0)*cos(psi0), -sin(phi0)*cos(psi0)+cos(phi0)*sin(theta0)*sin(psi0), cos(phi0)*cos(theta0)];
% BoomInfo.Mecc.I_rho = 3*BoomInfo.Mecc.I_rho;

% BoomInfo.Mecc.I_rho =10^-4*[ 14 8 0; 8 14 0; 0 0 27];
% BoomInfo.Mecc.m=0.075;
% U=[-15 0 0];
% Om=[ 0 0 10*2*pi];
P_tot=BoomInfo.Geom3D.Profile;
figure()
for i=1:2*(num+p_c)-2
    P_ruot=Tl_0'*(P_tot(3*i-2:3*i,:)-CG');
    plot3(P_ruot(1,:),P_ruot(2,:),P_ruot(3,:),'k');
    hold on
    axis equal
    grid on
end
%
C_fin_rot=BoomInfo.Geom3D.C_aer;

% Calcolo velocità in questi punti
P_dx=C_fin_rot(:,1:num)';
P_sx=C_fin_rot(:,num+2*p_c:end)';
for i=1:num
V_dx(i,:)=U'+cross(Om,P_dx(i,:))';
V_sx(i,:)=U'+cross(Om',P_sx(i,:))';
% V_dx(i,:)=V_dx(i,:)/norm(V_dx(i,:));
% V_sx(i,:)=V_sx(i,:)/norm(V_sx(i,:));

end
V_dx
V_sx
P_dx=(Tl_0'*P_dx')';
P_sx=(Tl_0'*P_sx')';
V_dx=(Tl_0'*V_dx')';
V_sx=(Tl_0'*V_sx')';

h1=quiver3(0.0,0,0,1,0 ,0,'b','linewidth',1);
quiver3(0.0,0,0,0,1 ,0,'b','linewidth',1);
quiver3(0.0,0,0,0,0 ,1,'b','linewidth',1);
quiver3([P_dx(:,1); P_sx(:,1)],[P_dx(:,2) ;P_sx(:,2)],[P_dx(:,3) ;P_sx(:,3)],[V_dx(:,1) ;V_sx(:,1)],[V_dx(:,2); V_sx(:,2)],[V_sx(:,3); V_sx(:,3)],'r','linewidth',1);
%quiver3(P_sx(:,1),P_sx(:,2),P_sx(:,3),V_sx(:,1),V_sx(:,2),V_sx(:,3),'r','linewidth',1);

plot3([0 -1],[0 0], [0 0],'--b','linewidth',1);
plot3([0 0],[0 -1], [0 0],'--b','linewidth',1);
plot3([0 0],[0 0], [0 -1],'--b','linewidth',1);
xlabel('X','fontsize',10,'interpreter','latex');
ylabel('Y','fontsize',10,'interpreter','latex');
zlabel('Z','fontsize',10,'interpreter','latex');
title('Configurazione Iniziale','fontsize',11,'interpreter','latex');
xlim([-2*l,2*l]);
Us=Tl_0'*U';
Us=Us/norm(Us);
Oms=Tl_0'*Om';
Oms=Oms/norm(Oms);
% h4=quiver3(CG(1),CG(2),CG(3),Us(1),Us(2),Us(3),'g','linewidth',1);
% h5=quiver3(CG(1),CG(2),CG(3),Oms(1),Oms(2),Oms(3),'c');
h4=quiver3(0,0,0,Us(1),Us(2),Us(3),'g','linewidth',1);
h5=quiver3(0,0,0,Oms(1),Oms(2),Oms(3),'c');

legend([h1 h4 h5],{'Asse-Terreno','V','Om'},'fontsize',8,'interpreter','latex');
set(gca,'TickLabelInterpreter','latex')
