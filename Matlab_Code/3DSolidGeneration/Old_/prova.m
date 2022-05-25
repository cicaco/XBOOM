%% Example Title
% Summary of example objective
clear all
close all
z=0.0000005;
P=[ 0 0 0; 1 0 0; 1 1 0; 0 1 0; 0 0 z; 1 0 z; 1 1 z; 0 1 z; 0 0 0];
% 1 2 3 4 5 6 7 8
T=[ 1 2 6; 1 6 5; 2 3 7; 2 7 6; 3 4 7; 4 8 7; 1  5 4; 5 8 4; 5  6 7; 5 7 8; 1 3 2; 1 4 3];
figure()
for i=1:12
p=fill3(P(T(i,:),1),P(T(i,:),2),P(T(i,:),3),'r');
hold on
p(1).FaceAlpha = 0.5;

end
TR=triangulation(T,P);
RBP=RigidBodyParams(TR)
axis 
