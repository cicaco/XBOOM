function [F,M,F_body,Alpha] = AeroLore(p,q,r,ux,uy,uz,Geom)

Aerc_tras=Geom.Aerc_tras;
Plant3D=Geom.Plant3D;
P_aerc=[Aerc_tras(1,:)' Aerc_tras(2,:)'   zeros(size(Aerc_tras(2,:)))'];

V_aerc=zeros(200,3);
V_inf=zeros(200,2);
C_section=[fliplr((fliplr(Plant3D(1,101:200))-(Plant3D(1,1:100)))) (fliplr(Plant3D(1,101:200))-(Plant3D(1,1:100)))];
Y_section=Aerc_tras(2,:);
rho_air=1.225;
R_aer=zeros(3,3,199);
F_aer=zeros(3,1,199);
XY_ac=zeros(3,199);
for i=2:100
    
    
    omega=[ p q r];
    V_aerc(i,:)=[ux;uy;uz;]'+cross(omega,P_aerc(i,:));
%     quiver3(P_aerc(i,1),P_aerc(i,2),P_aerc(i,3),V_aerc(i,1),V_aerc(i,2),V_aerc(i,3),0.01,'b');
    V_inf(i,1)=V_aerc(i,1);
    V_inf(i,2)=V_aerc(i,3);
    V=sqrt(V_inf(i,1)^2+V_inf(i,2)^2);
    alpha=atan2(V_inf(i,2)/V,V_inf(i,1)/V);
    Alpha(i-1)=(alpha)*180/pi;
    Sup=C_section(i)*(Y_section(i-1)-Y_section(i));
    Cl=@(A) 2*pi*A*(abs(A*180/pi)<6)+0;
    L=1/2*rho_air*V^2*Cl(alpha)*Sup;
    R_aer(:,:,i-1)=[cos(alpha) 0 -sin(alpha) ; 0 1 0; -sin(alpha) 0 cos(alpha) ];
    F_aer(:,:,i-1)=[0; 0 ; L];
    XY_ac(:,i-1)=[(Aerc_tras(1,i)+Aerc_tras(1,i-1))/2 (Aerc_tras(2,i)+Aerc_tras(2,i-1))/2 0];
    XY_ac(:,i-1)=[P_aerc(i,1) P_aerc(i,2) P_aerc(i,3)];
    %M_aer(:,:,i-1)=cross([Aerc_tras(1,i) Aerc_tras(2,i) 0]',[0; 0 ; L]);
end
F_body=reshape((pagemtimes(R_aer,F_aer)),[3,199]);
M_body=cross(XY_ac',F_body')';

F=sum(F_body,2);
M=sum(M_body,2);


