clear all
close all
load('P3D.mat');

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
figure()

hold on
for i=1:numel(T)
    text(P3d_i(i,1),P3d_i(i,2),P3d_i(i,3),string(i));
plot3(P3d_i(:,1),P3d_i(:,2),P3d_i(:,3),'*r');
end