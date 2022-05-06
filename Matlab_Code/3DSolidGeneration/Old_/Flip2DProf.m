clear all
close all
Profile2D=importdata('Naca0012.dat');
Xp_2d_b=[Profile2D.data(2:67,1) ; fliplr(Profile2D.data(68:end,1)')'];
Zp_2d_b=fliplr([Profile2D.data(2:67,2) ; fliplr(Profile2D.data(68:end,2)')']);
Chord=1;
%Cambio sistema di riferimento traslazione sull asse x di chord/2
Xp_flip=-(-Chord/2.*ones(size(Xp_2d_b))+Xp_2d_b)+Chord/2.*ones(size(Xp_2d_b));

Zp_flip=(Zp_2d_b);
[n,m]=size(Xp_2d_b);
%Clock-wise direction regeneration
Xp_flip=[fliplr(Xp_flip(1:n/2)')';fliplr(Xp_flip(n/2+1:end)')'];
Zp_flip=[fliplr(Zp_flip(1:n/2)')';fliplr(Zp_flip(n/2+1:end)')'];
figure()
plot(Xp_2d_b,Zp_2d_b,'*k','linewidth',1);
grid on
hold on
plot(Xp_flip,Zp_flip,'*r','linewidth',1);
axis equal
ylabel('$Z_{b}$','fontsize',10,'interpreter','latex')
xlabel('$X_{b}$','fontsize',10,'interpreter','latex')

legend({'Naca0012'},'fontsize',8,'interpreter','latex')

%Creo tra i due profili una sorta di transizione dei profili

%Zp_flip=[Zp_flip(n/2+1:end) ;fliplr(Zp_flip(1:n/2))];
Xp_dorso=Xp_2d_b(1:n/2);
Xp_ventre=Xp_2d_b(1+n/2:end);
Zp_dorso=Zp_2d_b(1:n/2);
Zp_ventre=Zp_2d_b(1+n/2:end);
Xp_dorso_flip=Xp_flip(1:n/2);
Xp_ventre_flip=Xp_flip(1+n/2:end);
Zp_dorso_flip=Zp_flip(1:n/2);
Zp_ventre_flip=Zp_flip(1+n/2:end);

num=10; %Numero di profili si cui vuoi la transizione
Xp_2d_trans=zeros(num,n);
Zp_2d_trans=zeros(num,n);


for i=1:n/2
    Xp_2d_trans(:,i)=Xp_dorso(i).*ones(num,1);
    Zp_2d_trans(:,i)=linspace(Zp_dorso_flip(i),Zp_dorso(i),num)';
    Xp_2d_trans(:,i+n/2)=Xp_ventre(i).*ones(num,1);
    Zp_2d_trans(:,i+n/2)=linspace(Zp_ventre_flip(i),Zp_ventre(i),num)';
    
end
% 
% figure()
% for i=1:num
%      
%      plot(Xp_2d_trans(i,:),Zp_2d_trans(i,:));
% hold on
% end
% axis equal
