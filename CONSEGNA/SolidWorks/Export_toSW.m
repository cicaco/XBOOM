clear all
close all
load('P3D.mat'); %Importare i profili del boomerang da BoomInfo.Geom3D.Profile

%%
num=5;
p_c=15;
n=2*(num+p_c)-2;
T=1:1:132;
N=1:n;
N=[ 1 num num+1:3:num+p_c num+p_c+1 num+p_c+1+3:3:num+2*p_c n]

for i=N
    i
filename=strjoin(['ProfileSW\',string(i),'.txt']);
P3d_i=[P_tot(3*i-2:3*i,:)';P_tot(3*i-2:3*i,1)'].*1000;

writematrix(P3d_i,filename)
end
