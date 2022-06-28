% clear all
clc
close all

T=readtable('debugReNew.txt');
time=T.Var4;
Re=T.Var2;

meanRe=mean(Re);

figure()
plot(time,Re);
hold on
plot(time,meanRe.*ones(length(time),1),'--r')
xlabel('time');
ylabel('Re');
title('Re lungo la traiettoria calcolato nella stazione a 3/4 di apertura');