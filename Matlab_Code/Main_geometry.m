%% Script per creare la Geometria 3D
clear all 
close all
addpath(genpath('Geometry'));

%% Input Data
Chord=0.0488;
p_c=20; % numero di profili di "Transizione" nella parte centrale
l=0.3; % lunghezza della pala avente un profilo 2D definito, NON corrisponde alla lunghezza del boomerang
delta=30*pi/180; %Angolo di freccia
beta=0*pi/180; %Angolo di Diedro
pitch=0*pi/180; %Pitch angle
num=40; %Numero di profili totale su ciascuna metà;
PARA=2; %Parametro che permette di modificare la curvatura centrale (più si avvicna ad 1 pù dietro forma una V
% Profile 2D Shape
Profile2D=importdata('Naca0012.dat');
%% Profilo 2D flip e analisi
Xp=-[Profile2D.data(2:67,1) ; fliplr(Profile2D.data(68:end,1)')'].*Chord;
Zp=[Profile2D.data(2:67,2) ; fliplr(Profile2D.data(68:end,2)')'].*Chord;
Xp_flip=-(Chord/2.*ones(size(Xp))+Xp)+Chord/2.*ones(size(Xp))-Chord;
Zp_flip=(Zp);
figure()
plot(Xp,Zp,'*r');
hold on
plot(Xp_flip,Zp_flip,'oc');
axis equal
set(gca,'Xdir','reverse')
[n,~]=size(Xp);
%Clock-wise direction regeneration
Xp_flip=[fliplr(Xp_flip(1:n/2)')';fliplr(Xp_flip(n/2+1:end)')'];
Zp_flip=[fliplr(Zp_flip(1:n/2)')';fliplr(Zp_flip(n/2+1:end)')'];
%% Dati per il calettamento lungo la pala e la rotazione
% Divisione del boomernag come segue:
% Paladx(y negative) 1:num-p_c
% Parte centrale num-pc:num+p_c
% Palasx(ypositive) num+p_c:end
%Creo i profili di transizione per la parte centrale
[X_trans,Z_trans] = Profile2d_Trans(Xp,Zp_flip,Zp,p_c*2-1);
%Considero una matrice di dimensione 2*num-1*132 che rappresenta il profilo
%di ciascuna delle sezione del boomerang
X_2d_i=[ones(num-p_c,1)*Xp';X_trans;ones(num-p_c,1)*Xp_flip'];
Z_2d_i=[ones(num-p_c,1)*Zp';Z_trans;ones(num-p_c,1)*Zp_flip'];

%Posizione del centro aerodinamico delle 2*num-1 sezioni (Pala dx: 1/4 Pala
%Sx 3/4
Fract=[1/4*ones(1,num-p_c) linspace(1/4,3/4,2*p_c-1) 3/4*ones(1,num-p_c)];

%Angoli di Freccia,Diedro, Pitch di ogni sezione
D_i=[-delta.*ones(1,num-p_c) linspace(-delta,delta,2*p_c-1) delta.*ones(1,num-p_c)];
B_i=[-beta.*ones(1,num-p_c) linspace(-beta,beta,2*p_c-1) beta.*ones(1,num-p_c)];
P_i=[-pitch.*ones(1,num-p_c) linspace(-pitch,pitch,2*p_c-1) pitch.*ones(1,num-p_c)];

%Angoli di Freccia e Diedro per trovare correttamente la posizione finale
%del centro aerodinamico
D_i_aer=[-delta.*ones(1,num-1) 0 delta.*ones(1,num-1)];
B_i_aer=[-beta.*ones(1,num) beta.*ones(1,num-1)];

%% Trovo la posizione finale del centro aerodinamico di ogni sezione

C=linspace(0,-Chord,num);
P=[C;zeros(1,num);zeros(1,num)];
U_Delta=[0 0 1];
li=linspace(l,0.1*l,num-p_c);
Tx=[ -sin(delta).*li zeros(1,p_c*2-1) -sin(delta).*fliplr(li)];
Ty=[ -cos(delta).*li zeros(1,p_c*2-1) cos(delta).*fliplr(li)];
Tz=zeros(1,numel(Tx));
P_r=[];
P_center_r=[];

figure()
for i=1:2*num-1
    
    [P_1] = Rot_Point([0 0 1],D_i(i),P,[C(end)*PARA;0;0]); %Rotazione rispetto all'asse z, rispetto però al "bordo di uscita" del profilo
    P_1(1,:)=P_1(1,:)+Tx(i);
    P_1(2,:)=P_1(2,:)+Ty(i);
    [C_aer] = AerCenter(P_1,Chord,Fract(i));
    %plot(P_1(1,:),P_1(2,:),'r');
    %text(P_1(1,1),P_1(2,1),string(i))
    hold on
    U_beta= Rot_Point(U_Delta,D_i_aer(i),[1; 0 ;0],[Chord 0 0]'); %devo ottenere il versore ruotato da x a x2 (body)
    [C_aer_rot] = Rot_Point(U_beta,B_i_aer(i),C_aer,[0;0;0]);
    
    C_fin(:,i)=C_aer;
    C_fin_rot(:,i)=C_aer_rot;
    %Adesso devo ruotare la pala
end
set(gca,'Ydir','reverse')
set(gca,'Xdir','reverse')
axis equal
figure()
C_fin_rot(2,num)=0;
plot3(C_fin_rot(1,:),C_fin_rot(2,:),C_fin_rot(3,:),'ob');
hold on
plot3(C_fin(1,:),C_fin(2,:),C_fin(3,:),'*r');

set(gca,'Ydir','reverse')
set(gca,'Xdir','reverse')
axis equal
%% Creazione della nuova di punti che compongono il boomerang
%Ruoto ciascun profilo e lo traslo nel centro aerodinamico calcolato in
%precedenza
% Capire se si puù fare una parte centrale un po migliore
P_tot=[];
for i=1:num*2-1
    P_2d=[X_2d_i(i,:);zeros(size(Z_2d_i(i,:)));Z_2d_i(i,:)];
    [P_fin] = Rot_Point([0 0 1],D_i(i),P_2d,[Fract(i)*C(end);0;0]); %Rotazione rispetto all'asse z, rispetto però al "bordo di uscita" del profilo
    U_beta= Rot_Point(U_Delta,D_i(i),[1; 0 ;0],[Fract(i)*C(end);0;0]); %devo ottenere il versore ruotato da x a x2 (body)
    [P_fin] = Rot_Point(U_beta,B_i(i),P_fin,[Fract(i)*C(end);0;0]);
    U_pitch_d= Rot_Point(U_Delta,D_i(i),[0; 1; 0 ],[Fract(i)*C(end);0;0]); % devo ottenre il versore ruotato da [0 1 0], ruotato di delta e poi di beta
    U_pitch= Rot_Point(U_beta,B_i(i),U_pitch_d,[Fract(i)*C(end);0;0]);
    [P_fin] = Rot_Point(U_pitch ,P_i(i),P_fin,[Fract(i)*C(end);0;0]);
    
    % Fino a num sono sulla pala destra e il centro aerodinamico e c/4
    %     sigma=D_i(i);
    %     coning=B_i(i);
    %     pitch=P_i(i);
    %     Tj1=[sin(sigma)*cos(pitch)+cos(sigma)*sin(coning)*sin(pitch), -cos(sigma)*cos(pitch)+sin(sigma)*sin(coning)*sin(pitch), -cos(coning)*sin(pitch)
    %     cos(sigma)*cos(coning), sin(sigma)*cos(coning), +sin(coning)
    %     sin(sigma)*sin(pitch)-cos(sigma)*sin(coning)*cos(pitch), -cos(sigma)*sin(pitch)-sin(sigma)*sin(coning)*cos(pitch), cos(coning)*cos(pitch)];
    
    %     P_fin=Tj1'*P_2d;
    
    P_fin(1,:)=P_fin(1,:)-Fract(i)*C(end)+C_fin_rot(1,i);
    P_fin(2,:)=P_fin(2,:)+C_fin_rot(2,i);
    P_fin(3,:)=P_fin(3,:)+C_fin_rot(3,i);
%     figure(11)
%         plot3(P_fin(1,:),P_fin(2,:),P_fin(3,:),'*r');

    if i~=num
        P_tot=[P_tot;P_fin];
    end
end


%% Carattersitiche inerziali
% Devo riportare il cad rispetto al baricentro (Solo posizione)

figure()
for i=1:2*num-2
    plot3(P_tot(3*i-2,:),P_tot(3*i-1,:),P_tot(3*i,:),'*r');
    hold on
    axis equal
end
% Start assembly each solid
figure()
n_prec=0;
tr=[];
xyz=[];
warning('off','all')
% calcolo il centro di massa per ogni tasca
m_tot = 0;


Cg_tot=zeros(1,3);
Ixx = 0;
Iyy = 0;
Izz = 0;
Ixy = 0;
Izy = 0;
Ixz = 0;
for i=2:2*num-2
    P_prec=P_tot(3*(i-1)-2:3*(i-1),:);
    P_succ=P_tot(3*i-2:3*i,:);
    figure(10)
    plot3(P_prec(1,:),P_prec(2,:),P_prec(3,:),'*r');
    hold on
    plot3(P_succ(1,:),P_succ(2,:),P_succ(3,:),'ob');
    if norm(P_prec-P_succ)~=0
        P_i=[P_prec';P_succ'];
        shp = alphaShape(P_i,1);
        [tr_i, xyz_i] = boundaryFacets(shp);
        
        n_succ=length(xyz_i)+n_prec;
        tr=[tr;tr_i+n_prec];
        xyz=[xyz;xyz_i];
        n_prec=n_succ;
        figure(11)
        plot(shp)
        hold on
        
        %Calcolo baricentro
        RBP=RigidBodyParams(triangulation(tr_i,xyz_i));
        I_i=RBP.inertia_tensor;
        CG_i=RBP.centroid;
        m_i=RBP.volume;
        CG_tasche(:,i)=CG_i;
        d=1; %densità unitaria
        XCg_tot = (m_tot * Cg_tot(1) + m_i * CG_i(1)) / (m_tot + m_i);
        YCg_tot = (m_tot * Cg_tot(2) + m_i * CG_i(2)) / (m_tot + m_i);
        ZCg_tot = (m_tot * Cg_tot(3) + m_i * CG_i(3)) / (m_tot + m_i);
        dX = XCg_tot - Cg_tot(1);
        dY = YCg_tot - Cg_tot(2);
        dZ = ZCg_tot - Cg_tot(3);
        dX_i = XCg_tot - CG_i(1);
        dY_i = YCg_tot - CG_i(2);
        dZ_i = ZCg_tot - CG_i(3);
        
        Ixx = Ixx + d * I_i(1,1) + m_tot * (dY * dY + dZ * dZ) + m_i * (dY_i * dY_i + dZ_i * dZ_i);
        Iyy = Iyy + d * I_i(2,2) + m_tot * (dX * dX + dZ * dZ) + m_i * (dX_i * dX_i + dZ_i * dZ_i);
        Izz = Izz + d * I_i(3,3) + m_tot * (dX * dX + dY * dY) + m_i * (dY_i * dY_i + dX_i * dX_i);
        Ixy = Ixy + d * I_i(1,2) + m_tot * dX * dY + m_i * dX_i * dY_i;
        Izy = Izy + d * I_i(2,3) + m_tot * dZ * dY + m_i * dZ_i * dY_i;
        Ixz = Ixz + d * I_i(1,3) + m_tot * dZ * dX + m_i * dZ_i * dX_i;
        m_tot = m_i + m_tot;
        
        Cg_tot(1) = XCg_tot;
        Cg_tot(2) = YCg_tot;
        Cg_tot(3) = ZCg_tot;
    end
end
warning('on','all')

pr_fin=triangulation(tr,xyz);


RBP=RigidBodyParams(pr_fin);
CG=RBP.centroid
 
xyz_CG=[xyz(:,1)-CG(1) xyz(:,2)-CG(2) xyz(:,3)-CG(3)];
pr_CG=triangulation(tr,xyz_CG);
RBP_CG=RigidBodyParams(pr_CG);
m=0.13
Rho=m/RBP_CG.volume;
I=Rho*RBP_CG.inertia_tensor
  
CG=RBP_CG.centroid;
 %Stima baricentro cannata totalmente (Forse le tasche danno fastidio)?
 % Magari si può calcolare dentro direttamente
%% Creazione file Stl 
filename=['Boom_D'+string(delta*180/pi)+'_B'+string(beta*180/pi)+'_P'+string(pitch*180/pi)+'.stl'];
stlwrite(pr_CG, filename);

%% Aerodinamica
%per le caratterisitche aerodinamiche in C_fin_rot sono presenti le 
%posizioni dei centri aerodinamici (non sono nel riferimento body)
figure()
    plot(shp)
% Ce forse creando queste tasche fa qualche puttanata strana sto programma,
% magari va solo utilizzato il teorema del trasporto e torna più sensato
% Niente esce allo stesso modo........ assurdo
hold on
plot3(CG_tasche(1,:),CG_tasche(2,:),CG_tasche(3,:),'*r');