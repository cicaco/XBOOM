function [Xp_2d_trans,Zp_2d_trans] = Profile2d_Trans(X1,Z1,Z2,num)
%PROFILE2D_TRANS Summary of this function goes here
%   Detailed explanation goes here
%Cambio sistema di riferimento traslazione sull asse x di chord/2


%Creo tra i due profili una sorta di transizione dei profili
[n,~]=size(X1);
%Zp_flip=[Zp_flip(n/2+1:end) ;fliplr(Zp_flip(1:n/2))];
Xp_dorso=X1(1:n/2);
Xp_ventre=X1(1+n/2:end);
Zp_dorso=Z1(1:n/2);
Zp_ventre=Z1(1+n/2:end);

Zp_dorso_flip=Z2(1:n/2);
Zp_ventre_flip=Z2(1+n/2:end);

 %Numero di profili si cui vuoi la transizione
Xp_2d_trans=zeros(num,n);
Zp_2d_trans=zeros(num,n);


for i=1:n/2
    Xp_2d_trans(:,i)=Xp_dorso(i).*ones(num,1);
    Zp_2d_trans(:,i)=linspace(Zp_dorso_flip(i),Zp_dorso(i),num)';
    Xp_2d_trans(:,i+n/2)=Xp_ventre(i).*ones(num,1);
    Zp_2d_trans(:,i+n/2)=linspace(Zp_ventre_flip(i),Zp_ventre(i),num)';
    
end
end

