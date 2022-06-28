clear all
close all
load('P3D.mat');
%%
Profile2D=importdata('Naca0012.dat');
Chord=0.05;
Xp=-[Profile2D.data(2:67,1) ; fliplr(Profile2D.data(68:end,1)')'].*Chord;
Zp=[Profile2D.data(2:67,2) ; fliplr(Profile2D.data(68:end,2)')'].*Chord;
% Profile2D=importdata('Naca0020.dat');
% Xp=-[0; fliplr(Profile2D(1:65,1)) ; fliplr(Profile2D(66:end,1)')'].*Chord;
% Zp=[0 ;fliplr(Profile2D(1:65,2)) ; fliplr(Profile2D(66:end,2)')'].*Chord;
Xp_flip=-(Chord/2.*ones(size(Xp))+Xp)+Chord/2.*ones(size(Xp))-Chord;
Zp_flip=(Zp);
figure()
hold on
for i=1:132
    text(Xp(i),Zp(i),string(i));
end
plot(Xp,Zp,'*r');
hold on
figure()
hold on
for i=1:132
    text(Xp_flip(i),Zp_flip(i),string(i));
end
 plot(Xp_flip,Zp_flip,'oc');
 axis equal
set(gca,'Xdir','reverse')
[n,~]=size(Xp);
%Clock-wise direction regeneration
Xp_flip=[fliplr(Xp_flip(1:n/2)')';fliplr(Xp_flip(n/2+1:end)')'];
Zp_flip=[fliplr(Zp_flip(1:n/2)')';fliplr(Zp_flip(n/2+1:end)')'];
%%
num=20;
p_c=20;
n=2*(num+p_c)-2;
T=1:3:132;
N=1:n;
N=[ 1 num num+1:3:num+p_c num+p_c+1 num+p_c+1+3:3:num+2*p_c n]

for i=N
    i
filename=strjoin(['ProfileSW\',string(i),'.txt']);
P3d_i=[P_tot(3*i-2:3*i,T)';P_tot(3*i-2:3*i,1)'].*1000;

writematrix(P3d_i,filename)
end
figure()

hold on
for i=1:numel(T)
    text(P3d_i(i,1),P3d_i(i,2),P3d_i(i,3),string(i));
plot3(P3d_i(:,1),P3d_i(:,2),P3d_i(:,3),'*r');
end