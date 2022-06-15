%Initial Condition
function InitialConditionPlot(Tl_0,T0,ustart,Om,BoomInfo)
% U,Om sistema body
num=BoomInfo.Geom3D.num;
p_c=BoomInfo.Geom3D.p_c;
CG=BoomInfo.Mecc.CG;
l=BoomInfo.Pianta.l;

% BoomInfo.Mecc.I_rho =10^-4*[ 14 8 0; 8 14 0; 0 0 27];
% BoomInfo.Mecc.m=0.075;
% U=[-15 0 0];
% Om=[ 0 0 10*2*pi];
P_tot=BoomInfo.Geom3D.Profile;

figure()
for i=1:2*(num+p_c)-2
    P_tot(3*i-2:3*i,:)=P_tot(3*i-2:3*i,:)-CG';
    P_ruot=Tl_0'*T0'*P_tot(3*i-2:3*i,:);
    plot3(P_ruot(1,:),P_ruot(2,:),P_ruot(3,:),'k');
    hold on
    axis equal
    grid on
end
h1=quiver3(0.0,0,0,1,0 ,0,'b','linewidth',1);
quiver3(0.0,0,0,0,1 ,0,'b','linewidth',1);
quiver3(0.0,0,0,0,0 ,1,'b','linewidth',1);
plot3([0 -1],[0 0], [0 0],'--b','linewidth',1);
plot3([0 0],[0 -1], [0 0],'--b','linewidth',1);
plot3([0 0],[0 0], [0 -1],'--b','linewidth',1);
xlabel('X','fontsize',10,'interpreter','latex');
ylabel('Y','fontsize',10,'interpreter','latex');
zlabel('Z','fontsize',10,'interpreter','latex');
title('Configurazione Iniziale','fontsize',11,'interpreter','latex');
xlim([-2*l,2*l]);
Us=Tl_0'*T0'*ustart;
Us=Us/norm(Us);
Oms=Tl_0'*T0'*Om;
Oms=Oms/norm(Oms);
h4=quiver3(0,0,0,Us(1),Us(2),Us(3),'g','linewidth',1);
h5=quiver3(0,0,0,Oms(1),Oms(2),Oms(3),'c');

legend([h1 h4 h5],{'Asse-Terreno','V','Om'},'fontsize',8,'interpreter','latex');
set(gca,'TickLabelInterpreter','latex')
