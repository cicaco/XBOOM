function [vel_ind]=VelInd_FAST2(u,omega,v_ind_old,BoomInfo)

alpha_cl=BoomInfo.Aero.alpha_cl;
alpha_cd=BoomInfo.Aero.alpha_cd;
CL_t=BoomInfo.Aero.Cl;
CD_t=BoomInfo.Aero.Cd;
psi_ciclo=linspace(0,2*pi,5);
psi_vect=midspan(psi_ciclo);

%% import geometria
c=BoomInfo.Profile.Chord; %m
L=BoomInfo.Pianta.l;
freccia=BoomInfo.Pianta.freccia;
coning=BoomInfo.Pianta.diedro; %(rot asse x2)
pitch=BoomInfo.Pianta.pitch; %(rot asse y3)
%pala sx (denominata 1)
xac1=BoomInfo.Aero.P_origin_Sx(1); %posizione centro aerodinamico pala 1
start1=BoomInfo.Aero.Start_Sx; %inizio pala 1 con profilo costante
%pala2
xac2=BoomInfo.Aero.P_origin_Dx(1);%posizione centro aerodinamico pala 2
start2=BoomInfo.Aero.Start_Dx; %inizio pala 2 con profilo costante

%% definizione sistemi di riferimento pala
%PALA 1
span1=linspace(start1,start1+L,10);
eta1=midspan(span1);
lambda=pi/2+freccia;
%svergolamento
twist1=zeros(1,length(eta1)).*ones(1,length(eta1),length(psi_vect));
%spanwise wing distance

%calcolo n° Re
spanRe=3/4*(start1+L);

%matrice di rotazione da body a blade pala 1
Tj1=[sin(lambda)*cos(pitch)+cos(lambda)*sin(coning)*sin(pitch), -cos(lambda)*cos(pitch)+sin(lambda)*sin(coning)*sin(pitch), -cos(coning)*sin(pitch)
    cos(lambda)*cos(coning), sin(lambda)*cos(coning), +sin(coning)
    sin(lambda)*sin(pitch)-cos(lambda)*sin(coning)*cos(pitch), -cos(lambda)*sin(pitch)-sin(lambda)*sin(coning)*cos(pitch), cos(coning)*cos(pitch)];


%SECONDA PALA
%spanwise wing distance
span2=linspace(start2,start2+L,10);
eta2=midspan(span2);
lambda=2*pi-lambda;
%svergolamento
twist2=zeros(1,length(eta2)).*ones(1,length(eta2),length(psi_vect));

%matrice di rotazione da body a blade
Tj2=[sin(lambda)*cos(pitch)+cos(lambda)*sin(coning)*sin(pitch), -cos(lambda)*cos(pitch)+sin(lambda)*sin(coning)*sin(pitch), -cos(coning)*sin(pitch)
    cos(lambda)*cos(coning), sin(lambda)*cos(coning), +sin(coning)
    sin(lambda)*sin(pitch)-cos(lambda)*sin(coning)*cos(pitch), -cos(lambda)*sin(pitch)-sin(lambda)*sin(coning)*cos(pitch), cos(coning)*cos(pitch)];

F_ciclopsi=zeros(3,4);
% nps=5;
% reshape(cos(psi_vect),[1 1 nps])
% M_ciclopsi=[];
T_ciclopsi_vect=[reshape(cos(psi_vect),[1 1 length(psi_vect)]) reshape(sin(psi_vect),[1 1 length(psi_vect)]) zeros(1,1,length(psi_vect));...
    -reshape(sin(psi_vect),[1 1 length(psi_vect)]) reshape(cos(psi_vect),[1 1 length(psi_vect)]) zeros(1,1,length(psi_vect));...
    zeros(1,1,length(psi_vect)) zeros(1,1,length(psi_vect)) ones(1,1,length(psi_vect))];
xac1_ciclopsi_vect=reshape(pagemtimes(T_ciclopsi_vect,[xac1*ones(1,1,length(psi_vect)); zeros(1,1,length(psi_vect)); zeros(1,1,length(psi_vect))]),[1,3,length(psi_vect)]);
xac2_ciclopsi_vect=reshape(pagemtimes(T_ciclopsi_vect,[xac2*ones(1,1,length(psi_vect)); zeros(1,1,length(psi_vect)); zeros(1,1,length(psi_vect))]),[1,3,length(psi_vect)]);

ra1_vect=(xac1_ciclopsi_vect+(Tj1'*[zeros(1,numel(eta1));eta1;zeros(1,numel(eta1))])'.*ones(numel(eta1),3,length(psi_vect)));
ra2_vect=(xac2_ciclopsi_vect+(Tj2'*[zeros(1,numel(eta2));eta2;zeros(1,numel(eta2))])'.*ones(numel(eta2),3,length(psi_vect)));
omega_m_vect=[omega(1).*ones(numel(eta1),1) omega(2).*ones(numel(eta1),1) omega(3).*ones(numel(eta1),1)].*ones(numel(eta2),3,length(psi_vect));
U_vect=[u(1).*ones(numel(eta1),1) u(2).*ones(numel(eta1),1) u(3).*ones(numel(eta1),1)].*ones(numel(eta2),3,length(psi_vect));
vel1_vect=U_vect+cross(omega_m_vect,ra1_vect);
vel2_vect=U_vect+cross(omega_m_vect,ra2_vect);

w1_vect=pagemtimes(ones(3,3,length(psi_vect)).*Tj1, permute(-vel1_vect,[2 1 3])-[zeros(1,numel(eta1)) ;zeros(1,numel(eta1)); -v_ind_old.*ones(1,numel(eta1))].*ones(3,numel(eta1),length(psi_vect)));
w2_vect=pagemtimes(ones(3,3,length(psi_vect)).*Tj2, permute(-vel2_vect,[2 1 3])-[zeros(1,numel(eta2)) ;zeros(1,numel(eta2)); -v_ind_old.*ones(1,numel(eta2))].*ones(3,numel(eta2),length(psi_vect)));
AoA1_vect=wrapToPi(atan2(w1_vect(3,:,:),w1_vect(1,:,:))+twist1);
AoA2_vect=wrapToPi(atan2(w2_vect(3,:,:),w2_vect(1,:,:))+twist2);
CL_naca1_vect=interp1(alpha_cl, CL_t, AoA1_vect.*180/pi);
CL_naca2_vect=interp1(alpha_cl, CL_t, AoA2_vect.*180/pi);
CD_naca1_vect=interp1(alpha_cd, CD_t, AoA1_vect.*180/pi);
CD_naca2_vect=interp1(alpha_cd, CD_t, AoA2_vect.*180/pi);
net=length(eta1);

F2i_vect=(span2(2:net+1)-span2(1:net))'.*0.5.*1.225.*c.*permute(vecnorm(w2_vect([1 3],:,:),2).^2,[2 1 3]).*permute([-CL_naca2_vect.*sin(AoA2_vect)+CD_naca2_vect.*cos(AoA2_vect); zeros(size(AoA2_vect)); CL_naca2_vect.*cos(AoA2_vect)+CD_naca2_vect.*sin(AoA2_vect)],[2 1 3]);
F2i_vect=pagemtimes(ones(3,3,length(psi_vect)).*Tj2', permute(F2i_vect,[2 1 3]));
F1i_vect=(span1(2:net+1)-span1(1:net))'.*0.5.*1.225.*c.*permute(vecnorm(w1_vect([1 3],:,:),2).^2,[2 1 3]).*permute([-CL_naca1_vect.*sin(AoA1_vect)+CD_naca1_vect.*cos(AoA1_vect); zeros(size(AoA1_vect)); CL_naca1_vect.*cos(AoA1_vect)+CD_naca1_vect.*sin(AoA1_vect)],[2 1 3]);
F1i_vect=pagemtimes(ones(3,3,length(psi_vect)).*Tj1', permute(F1i_vect,[2 1 3]));

F_ciclopsi=permute(sum(F2i_vect,2)+sum(F1i_vect,2),[1,3,2]);


F_ciclopsi=fliplr(F_ciclopsi);
%calcolo v_indotta
S=2.28*10^-1;
vel_ind=sum(1./(2.*pi).*F_ciclopsi(3,:)./(2.*1.225.*S.*sqrt(u(1)^2+(u(3)-v_ind_old)^2)).*(psi_ciclo(2:length(psi_vect)+1)-psi_ciclo(1:length(psi_vect))));


end