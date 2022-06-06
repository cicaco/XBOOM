function [P_fin] = Rot_Point(u,T,P,o)
%ROT è una funzione che fornisce la matrice di rotazione da un asse e un
%angolo assegnato come input
% Input:
% u: 1x3 di norma 1
% T: Angolo in radianti
% P: punti da ruotare 3XN
% o: punto rispetto al quale ruotare 3X1

ux=u(1);
uy=u(2);
uz=u(3);
c=cos(T);
s=sin(T);

R=[ux^2*(1-c)+c ux*uy*(1-c)-uz*s ux*uz*(1-c)+uy*s;...
    ux*uy*(1-c)+uz*s uy^2*(1-c)+c uy*uz*(1-c)-ux*s;...
    ux*uz*(1-c)-uy*s uy*uz*(1-c)+ux*s uz^2*(1-c)+c];

[~,m]=size(P);
% Vuol dire che è un matrice 3X qiundi ho le coordinate scritte
% verticalmente
Trasl=o.*ones(1,m);
P_trasl=P-Trasl;
P_ruot=R*P_trasl;
P_fin=P_ruot+Trasl;

end

