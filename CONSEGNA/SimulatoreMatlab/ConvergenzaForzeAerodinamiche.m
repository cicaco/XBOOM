N=[5 10 15 20 60]

v=[0.263023503767109 0.263017163048167 0.263026690774737 0.263021631034916 0.263024876991859]
figure()
plot(N,v,'Linewidth',1.2);
grid on
xlabel('Numero intervalli in $0-360$','fontsize',11,'interpreter','latex')
ylabel('Velocit\`a indotta $[m s^{-1}]$ ','fontsize',11,'interpreter','latex')
title('\textbf{Convergenza velocit\`a indotta}','fontsize',12,'interpreter','latex')
set(gca,'TickLabelInterpreter','latex')
axis([0 60 0.2630 0.26304])

grid on
xlabel('r_0 [rad/s]','fontsize',11,'interpreter','latex')
ylabel('$\phi$ $[\ang]$','fontsize',11,'interpreter','latex')
title('\textbf{Area }','fontsize',12,'interpreter','latex')
set(gca,'TickLabelInterpreter','latex')