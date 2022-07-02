clear all
close all
load('27_06_GA.mat')
% i:D, j:L
[D_m,L_m]=meshgrid(Di,li);
figure()
surf(L_m,D_m,Dist)
Xm_vect=[];
cont=1;
for i=1:10
    for j=1:10
        if Dist(i,j)<5
            Xm_vect=[Xm_vect permute(Xm(i,j,:),[3,1,2])];
            P(1,cont)=Di(i);
            P(2,cont)=In(1,2,10*(i-1)+j);
            P(3,cont)=li(j);
            cont=cont+1;
        end  
    end
end
figure()
plot3(P(1,:),P(2,:),Xm_vect(1,:),'*r');
xlabel('Angolo di Freccia');
ylabel('Ixx');
zlabel('r0');
grid on
% std(Xm_vect')
% mean(Xm_vect')