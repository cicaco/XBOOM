clear all
close all
P=[0 0 0; 10 0 0; 10 10 0; 0 10 0; 0 0 10; 10 0 10; 10 10 10; 0 10 10;];
shp = alphaShape(P);
[tr_i, xyz_i] = boundaryFacets(shp);
plot(shp)
RBP=RigidBodyParams(triangulation(tr_i,xyz_i));

load('Ris.mat');
subplot(1,2,1)
plot(n,reshape(I(1,1,:),[1 6])./(I(1,1,end)),'r','linewidth',1);
hold on
grid on
plot(n,reshape(I(2,2,:),[1 6])./(I(2,2,end)),'c','linewidth',1);
plot(n,reshape(I(3,3,:),[1 6])./(I(3,3,end)),'g','linewidth',1);
plot(n,reshape(I(1,2,:),[1 6])./(I(1,2,end)),'b','linewidth',1);
plot(n,reshape(I(1,3,:),[1 6])./(I(1,3,end)),'y','linewidth',1);
plot(n,reshape(I(2,3,:),[1 6])./(I(2,3,end)),'k','linewidth',1);
legend({'$I_{xx}$','$I_{yy}$','$I_{zz}$','$I_{xy}$','$I_{xz}$','$I_{yz}$'},'fontsize',8,'interpreter','latex');
xlabel('n','fontsize',11,'interpreter','latex');
set(gca,'TickLabelInterpreter','latex')
ylabel('Inertia [Kg $m^2$]','fontsize',11,'interpreter','latex');
subplot(1,2,2)
plot(n,m./m(end),'b','linewidth',1);
grid on
legend('Mass [Kg]','fontsize',8,'interpreter','latex');
xlabel('n','fontsize',11,'interpreter','latex');
set(gca,'TickLabelInterpreter','latex')
ylabel('Z','fontsize',11,'interpreter','latex');
