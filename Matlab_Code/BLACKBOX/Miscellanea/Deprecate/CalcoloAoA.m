% clear all
clc
close all

T=readtable('debugAoA.txt');
AoA=T.Var3;

figure()
histogram(AoA,360)
title('AoA pala sx, da metà pala alla tip')