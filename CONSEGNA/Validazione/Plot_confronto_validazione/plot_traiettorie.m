
clear all;
close all;
clc;


%% SCRIPT UTILIZZATO PER CONFRONTARE TRAIETTORIE SIMULATE E SPERIMENTALI

% summary_lancio_1
% Tempo di ritorno calcolato: 2.52719 s
% Massima distanza percorsa: 13.17335 m

% summary_lancio_2
% Tempo di ritorno calcolato: 2.75932 s
% Massima distanza percorsa: 12.34990 m

%% LANCIO 1
% carica output stato dinamica simulata
load traj_lancio_2.mat % può essere 1 o 2
% lancio 2
load YZVideo_29.txt
load XZVideo_29.txt
% lancio 1 
load YZVideo_48.txt
load XZVideo_48.txt
% carica le traiettorie simulate viste però in prospettiva
load Lancio_1_XZ_prospettiva.txt
load Lancio_1_YZ_prospettiva.txt
load Lancio_2_YZ_prospettiva.txt
load Lancio_2_XZ_prospettiva.txt


x_sim_1 = YOUT(:,10);
y_sim_1 = YOUT(:,11);
z_sim_1 = YOUT(:,12);

% plot traiettoria 3D simulata in prospettiva
figure()
plot3(YOUT(:,10),YOUT(:,11),YOUT(:,12),'b','linewidth',1.2)
hold on
plot3(3, 0, 0, 'ro', 'linewidth', 1.2 )
plot3(0, 0, 0, 'ro', 'linewidth', 1.2 )
legend('CG','fontsize',10,'interpreter','latex')
xlabel('X','fontsize',11,'interpreter','latex');
set(gca,'TickLabelInterpreter','latex')
ylabel('Y','fontsize',11,'interpreter','latex');
zlabel('Z','fontsize',11,'interpreter','latex');
title('Boomerang Traiectory in 3D space','fontsize',12,'interpreter','latex');
% prioezione traiettoria in prosperttiva
set(gca, 'Projection','perspective')
% posizione osservatorre
campos([0, -20, 1.8])
grid on;
axis equal
%% Lancio 1
%YZ confronto/ vista laterale
figure;
plot(Lancio_1_YZ_prospettiva(:,1), Lancio_1_YZ_prospettiva(:,2),'o', "LineWidth", 1.2)
hold on;
plot(YZVideo_48(:,1), YZVideo_48(:,2),'o' ,"LineWidth", 1.2)
plot(6, 0, 'mo')
grid on; 
axis equal;
legend('Traiettoria simulata', 'Lancio sperimentale in prospettiva','Riferimento','fontsize',10,'interpreter','latex')
set(gca,'TickLabelInterpreter','latex')
xlabel('Y','fontsize',11,'interpreter','latex');
ylabel('Z','fontsize',11,'interpreter','latex');
title('Traiettoria nel Boomerang in prospettiva nel piano YZ, lancio 1','fontsize',12,'interpreter','latex');

%XZ confronto/ vista frontale
figure;
%plot(x_sim_1, z_sim_1, "LineWidth", 1.2)
plot(Lancio_1_XZ_prospettiva(:,1), Lancio_1_XZ_prospettiva(:,2), 'o', "LineWidth", 1.2)
hold on;
plot(XZVideo_48(:,1), XZVideo_48(:,2), 'o', "LineWidth", 1.2)
plot(10, 0, 'mo')
grid on; 
axis equal;
legend('Traiettoria simulata', 'Lancio sperimentale', 'Riferimento','fontsize',10,'interpreter','latex')
set(gca,'TickLabelInterpreter','latex')
xlabel('X','fontsize',11,'interpreter','latex');
ylabel('Z','fontsize',11,'interpreter','latex');
title('Traiettoria nel Boomerang in prospettiva nel piano XZ, lancio 1','fontsize',12,'interpreter','latex');

%% LANCIO 2
%load traj_lancio_2.mat


x_sim_2 = YOUT(:,10);
y_sim_2 = YOUT(:,11);
z_sim_2 = YOUT(:,12);

%YZ confornto/ vista laterale
figure;
plot(Lancio_2_YZ_prospettiva(:,1), Lancio_2_YZ_prospettiva(:,2), 'o', "LineWidth", 1.2)
hold on;
plot(YZVideo_29(:,1), YZVideo_29(:,2),'o', "LineWidth", 1.2)
plot(6,0, 'mo')
grid on; 
axis equal;
legend('Traiettoria simulata', 'Lancio sperimentale in prospettiva','fontsize',10,'interpreter','latex')
set(gca,'TickLabelInterpreter','latex')
xlabel('Y','fontsize',11,'interpreter','latex');
ylabel('Z','fontsize',11,'interpreter','latex');
title('Traiettoria nel Boomerang in prospettiva nel piano YZ, lancio 2','fontsize',12,'interpreter','latex');

%XZ confronto/ vista frontale
figure;
plot(Lancio_2_XZ_prospettiva(:,1),Lancio_2_XZ_prospettiva(:,2), 'o', "LineWidth", 1.2)
hold on;
plot(XZVideo_29(:,1), XZVideo_29(:,2),'o', "LineWidth", 1.2)
plot(10, 0, 'mo')
grid on; 
axis equal;
legend('Traiettoria simulata', 'Lancio sperimentale in prospettiva','fontsize',10,'interpreter','latex')
set(gca,'TickLabelInterpreter','latex')
xlabel('X','fontsize',11,'interpreter','latex');
ylabel('Z','fontsize',11,'interpreter','latex');
title('Traiettoria nel Boomerang in prospettiva nel piano XZ, lancio 2','fontsize',12,'interpreter','latex');
