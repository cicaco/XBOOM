function [BoomInfo] = Prova3(BoomInfo,varargin) 
% La funzione permette di creare la dimensione tridimensionale del boomerang
% dati in input un file struct, conentente tutti i dati del boomerang.
% INPUT
% BoomInfo, struct contente:
% - BoomInfo.Profile.Chord
% - BoomInfo.Profile.Chord;
% - BoomInfo.Geom3D.p_c: numero di profili di "Transizione" nella parte centrale
% - BoomInfo.Pianta.l: lunghezza della pala
% - BoomInfo.Pianta.freccia: Angolo di freccia
% - BoomInfo.Pianta.diedro: Angolo di Diedro
% - BoomInfo.Pianta.pitch: Angolo di Pitch
% - BoomInfo.Geom3D.num: Numero di profili su ciascuna pala;
% - BoomInfo.Geom3D.PARA; %Parametro che permette di modificare la
%   curvatura centrale (Para=1 non ce la curvatura dietro)
% - BoomInfo.Profile.Xp_dx; %Profilo pala dx (Asse X)
% - BoomInfo.Profile.Zp_dx; %Profilo pala dx (Asse Z)
% - BoomInfo.Profile.Xp_sx; %Profilo pala sx (Asse X)
% - BoomInfo.Profile.Zp_sx; %Profilo pala sx (Asse Z)
% OUTPUT
% Vengono aggiunti alla struct nuove variabili:
% - BoomInfo.Aero.P_origin_Dx: Punto di "origine" della pala dx (HP: no curvatura centrale)
% - BoomInfo.Aero.P_Start_Dx: Punto di inizio della pala dx (HP: no curvatura centrale)
% - BoomInfo.Aero.P_Finish_Dx:  Punto di fine della pala dx (HP: no curvatura centrale)
% - BoomInfo.Aero.Start_Dx: Distanza tra "Origine" e inizio per pala dx
% - BoomInfo.Aero.P_origin_Sx:
% - BoomInfo.Aero.P_Start_Sx:
% - BoomInfo.Aero.P_Finish_Sx:
% - BoomInfo.Aero.Start_Sx:
% - BoomInfo.Mecc.V: Volume Boomerang
% - BoomInfo.Mecc.I: Inerzia Boomerang
% OPZIONI
% - 'Plot_figure': Vengono mostrate le figure 
% - 'Create_Stl': Viene creato il file sitl
% - 'Density_variation': Opzione che permmette di creare una distribuzione
%   di densità sul boomerang (ad esempio creando una parte centrale del
%   boomerang più pesante), in questo caso è necessario dare in input un
%   secondo file avendo tali dimensione 1X(Num-3), ESEMPIO:
%   Dens_i=[1000.*ones(1,num-p_c-1) 1500*ones(1,2*p_c-1) 1000.*ones(1,num-p_c-1)];
% - 'Info': Viene dichirato cosa sta facendo il codice
% 
%% Import Data from BoomInfo
Chord=BoomInfo.Profile.Chord;
p_c=BoomInfo.Geom3D.p_c;
l=BoomInfo.Pianta.l;
delta=BoomInfo.Pianta.freccia;
beta=BoomInfo.Pianta.diedro;
pitch=BoomInfo.Pianta.pitch;
num=BoomInfo.Geom3D.num;
PARA=BoomInfo.Geom3D.PARA;
Xp_dx=BoomInfo.Profile.Xp_dx;
Zp_dx=BoomInfo.Profile.Zp_dx;
Xp_sx=BoomInfo.Profile.Xp_sx;
Zp_sx=BoomInfo.Profile.Zp_sx;
num=num+p_c;
%% Set option of the function
C_fig=0;
C_stl=0;
C_VarDens=0;
C_info=0;
nVarargs = length(varargin);
i=1;
cont=0;
while i<=nVarargs

    switch varargin{i}
        case 'Plot_figure'
            C_fig=1;
        case 'Create_Stl'
            C_stl=0;
        case 'Density_variation'
            C_VarDens=1;
            i=i+1;
            Dens_i= varargin{i+1};
        case 'Info'
            C_info=1;
        otherwise
            error('Verificare di aver inserito le opzioni corrette ')
    end
    i=i+1;
end

%% Dati per il calettamento lungo la pala e la rotazione
% Divisione del boomernag come segue:
% Paladx(y negative) 1:num-p_c
% Parte centrale num-pc:num+p_c
% Palasx(ypositive) num+p_c:end
%Creo i profili di transizione per la parte centrale
[X_trans,Z_trans] = Profile2d_Trans(Xp_dx,Zp_sx,Zp_dx,p_c*2-1);
%Considero una matrice di dimensione 2*num-1*132 che rappresenta il profilo
%di ciascuna delle sezione del boomerang
X_2d_i=[ones(num-p_c,1)*Xp_dx';X_trans;ones(num-p_c,1)*Xp_sx'];
Z_2d_i=[ones(num-p_c,1)*Zp_dx';Z_trans;ones(num-p_c,1)*Zp_sx'];

%Posizione del centro aerodinamico delle 2*num-1 sezioni (Pala dx: 1/4 Pala
%Sx 3/4
Fract=[1/4*ones(1,num-p_c) linspace(1/4,3/4,2*p_c-1) 3/4*ones(1,num-p_c)];

%Angoli di Freccia,Diedro, Pitch di ogni sezione
D_i=[-delta.*ones(1,num-p_c-1) linspace(-delta,delta,2*p_c+1) delta.*ones(1,num-p_c-1)];
B_i=[-beta.*ones(1,num-p_c-1) linspace(-beta,beta,2*p_c+1) beta.*ones(1,num-p_c-1)];
P_i=[-pitch.*ones(1,num-p_c-1) linspace(-pitch,pitch,2*p_c+1) pitch.*ones(1,num-p_c-1)];

%Angoli di Freccia e Diedro per trovare correttamente la posizione finale
%del centro aerodinamico
D_i_aer=[-delta.*ones(1,num-1) 0 delta.*ones(1,num-1)];
B_i_aer=[-beta.*ones(1,num) beta.*ones(1,num-1)];

%% Trovo la posizione finale del centro aerodinamico di ogni sezione
if C_info==1
    fprintf('Start Computing Aerodynamic center \n');
end
C=linspace(0,-Chord,num);
P=[C;zeros(1,num);zeros(1,num)];
U_Delta=[0 0 1];
li=linspace(l,0,num-p_c);
% Va aggiunta anche una translazione del primo profilo pari a q
Tx=[ -sin(delta).*li zeros(1,p_c*2-1) -sin(delta).*fliplr(li)];
Ty=[ -cos(delta).*li zeros(1,p_c*2-1) cos(delta).*fliplr(li)];
Tz=zeros(1,numel(Tx));
P_r=[];
P_center_r=[];


for i=1:2*num-1

    [P_1] = Rot_Point([0 0 1],D_i(i),P,[C(end)*PARA;0;0]); %Rotazione rispetto all'asse z, rispetto però al "bordo di uscita" del profilo
    P_1(1,:)=P_1(1,:)+Tx(i);
    P_1(2,:)=P_1(2,:)+Ty(i);
    [C_aer] = AerCenter(P_1,Chord,Fract(i));

    U_beta= Rot_Point(U_Delta,D_i_aer(i),[1; 0 ;0],[Chord 0 0]'); %devo ottenere il versore ruotato da x a x2 (body)
    [C_aer_rot] = Rot_Point(U_beta,B_i_aer(i),C_aer,[0;0;0]);

    C_fin(:,i)=C_aer;
    C_fin_rot(:,i)=C_aer_rot;
end

%% Creazione della nuova di punti che compongono il boomerang
%Ruoto ciascun profilo e lo traslo nel centro aerodinamico calcolato in
%precedenza
% Capire se si puù fare una parte centrale un po migliore
P_tot=[];
if C_info==1
    fprintf('Start Computing Boomerang Shape by Points\n');
end
for i=1:num*2-1
    P_2d=[X_2d_i(i,:);zeros(size(Z_2d_i(i,:)));Z_2d_i(i,:)];
    [P_fin] = Rot_Point([0 0 1],D_i(i),P_2d,[Fract(i)*C(end);0;0]); %Rotazione rispetto all'asse z, rispetto però al "bordo di uscita" del profilo
    U_beta= Rot_Point(U_Delta,D_i(i),[1; 0 ;0],[Fract(i)*C(end);0;0]); %devo ottenere il versore ruotato da x a x2 (body)
    [P_fin] = Rot_Point(U_beta,B_i(i),P_fin,[Fract(i)*C(end);0;0]);
    U_pitch_d= Rot_Point(U_Delta,D_i(i),[0; 1; 0 ],[Fract(i)*C(end);0;0]); % devo ottenre il versore ruotato da [0 1 0], ruotato di delta e poi di beta
    U_pitch= Rot_Point(U_beta,B_i(i),U_pitch_d,[Fract(i)*C(end);0;0]);
    [P_fin] = Rot_Point(U_pitch ,P_i(i),P_fin,[Fract(i)*C(end);0;0]);


    P_fin(1,:)=P_fin(1,:)-Fract(i)*C(end)+C_fin_rot(1,i);
    P_fin(2,:)=P_fin(2,:)+C_fin_rot(2,i);
    P_fin(3,:)=P_fin(3,:)+C_fin_rot(3,i);

    if i~=num
        P_tot=[P_tot;P_fin];
    end
end


%% Carattersitiche inerziali
% Devo riportare il cad rispetto al baricentro (Solo posizione)
if C_fig==1
    figure(1)
    for i=1:2*num-2
        plot3(P_tot(3*i-2,:),P_tot(3*i-1,:),P_tot(3*i,:),'*r');
        hold on
        axis equal
    end
end
% Start assembly each solid

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
%Dens_i=[1000.*ones(1,num-p_c-1) 1500*ones(1,2*p_c-1) 1000.*ones(1,num-p_c-1)];
if C_info==1
    fprintf('Start Computing Boomerang inertial characteristics \n');
end
for i=2:2*num-2
    P_prec=P_tot(3*(i-1)-2:3*(i-1),:);
    P_succ=P_tot(3*i-2:3*i,:);
    if C_fig==1

        figure(2)
        plot3(P_prec(1,:),P_prec(2,:),P_prec(3,:),'*r');
        hold on
        plot3(P_succ(1,:),P_succ(2,:),P_succ(3,:),'ob');
    end
    if norm(P_prec-P_succ)~=0
        P_i=[P_prec';P_succ'];
        shp = alphaShape(P_i,1);
        [tr_i, xyz_i] = boundaryFacets(shp);

        n_succ=length(xyz_i)+n_prec;
        tr=[tr;tr_i+n_prec];
        xyz=[xyz;xyz_i];
        n_prec=n_succ;
        if C_fig==1

            figure(3)
            plot(shp)
            hold on
        end
        if C_VarDens==1
            %Calcolo baricentro
            RBP=RigidBodyParams(triangulation(tr_i,xyz_i));

            CG_i=RBP.centroid;
            d=Dens_i(i-1); %densità unitaria
            m_i=d*RBP.volume;
            I_i=d*RBP.inertia_tensor;
            CG_tasche(:,i)=CG_i;

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
end
Ixx 
Iyy 
Izz 
Ixy 
Izy 
Ixz  
warning('on','all')

pr_fin=triangulation(tr,xyz);


RBP=RigidBodyParams(pr_fin);
CG=RBP.centroid;

xyz_CG=[xyz(:,1)-CG(1) xyz(:,2)-CG(2) xyz(:,3)-CG(3)];
pr_CG=triangulation(tr,xyz_CG);
RBP_CG=RigidBodyParams(pr_CG);

I=RBP_CG.inertia_tensor;
V=RBP_CG.volume;


%Stima baricentro cannata totalmente (Forse le tasche danno fastidio)?
% Magari si può calcolare dentro direttamente
%% Creazione file Stl
if C_stl==1

    filename=['Boom_D'+string(delta*180/pi)+'_B'+string(beta*180/pi)+'_P'+string(pitch*180/pi)+'.stl'];
    stlwrite(pr_CG, filename);
    if C_info==1
        fprintf('STL file creation name: ');
        fprintf(filename);
        fprintf('\n');
    end

end
%% Aerodinamica
%per le caratterisitche aerodinamiche in C_fin_rot sono presenti le
%posizioni dei centri aerodinamici (non sono nel riferimento body)


Norm=(C_fin_rot(:,num+p_c)-C_fin_rot(:,num+p_c+1))/norm(C_fin_rot(:,num+p_c)-C_fin_rot(:,num+p_c+1));
[I_sx,~] = line_plane_intersection(Norm,C_fin_rot(:,num+p_c),[0 1 0]',[0 0 0]');


Norm=(C_fin_rot(:,num-p_c-1)-C_fin_rot(:,num-p_c))/norm(C_fin_rot(:,num-p_c-1)-C_fin_rot(:,num-p_c));
[I_dx,~] = line_plane_intersection(Norm,C_fin_rot(:,num-p_c),[0 1 0]',[0 0 0]');


I_origin_Sx=I_sx-CG';
I_Start_Sx=C_fin_rot(:,num-p_c)-CG';
I_Finish_Sx=C_fin_rot(:,1)-CG';

I_origin_Dx=I_dx-CG';
I_Start_Dx=C_fin_rot(:,num+p_c)-CG';
I_Finish_Dx=C_fin_rot(:,end)-CG';

Start_Dx=norm(I_Start_Dx-I_origin_Dx);
l_check_Dx=norm(-I_Start_Dx+I_Finish_Dx);
Start_Sx=norm(I_Start_Sx-I_origin_Sx);
l_check_Sx=norm(-I_Start_Sx+I_Finish_Sx);
if C_fig==1

    figure()
    plot3(C_fin_rot(1,:),C_fin_rot(2,:),C_fin_rot(3,:),'*r');
    quiver3(C_fin_rot(1,num+p_c),C_fin_rot(2,num+p_c),C_fin_rot(3,num+p_c),Norm(1),Norm(2),Norm(3));
    quiver3(C_fin_rot(1,num-p_c),C_fin_rot(2,num-p_c),C_fin_rot(3,num-p_c),Norm(1),Norm(2),Norm(3));

    hold on
    plot3(C_fin_rot(1,num+p_c),C_fin_rot(2,num+p_c),C_fin_rot(3,num+p_c),'*b');
    plot3(I_sx(1),I_sx(2),I_sx(3),'ok');
    plot3(I_dx(1),I_dx(2),I_dx(3),'ok');
    plot3(C_fin_rot(1,num-p_c),C_fin_rot(2,num-p_c),C_fin_rot(3,num-p_c),'*g');
    axis equal
    grid on
end
BoomInfo.Aero.P_origin_Dx=I_origin_Dx;
BoomInfo.Aero.P_Start_Dx=I_Start_Dx;
BoomInfo.Aero.P_Finish_Dx=I_Finish_Dx;
BoomInfo.Aero.Start_Dx=Start_Dx;
BoomInfo.Aero.P_origin_Sx=I_origin_Sx;
BoomInfo.Aero.P_Start_Sx=I_Start_Sx;
BoomInfo.Aero.P_Finish_Sx=I_Finish_Sx;
BoomInfo.Aero.Start_Sx=Start_Sx;
BoomInfo.Mecc.V=V;
BoomInfo.Mecc.I=I;
end

