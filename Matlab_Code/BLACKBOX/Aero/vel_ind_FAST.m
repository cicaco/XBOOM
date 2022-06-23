function [vel_ind]=vel_ind_FAST(u,omega,v_ind_old,BoomInfo)
alpha_t=BoomInfo.Aero.alpha_t;
CL_t=BoomInfo.Aero.Cl;
CD_t=BoomInfo.Aero.Cd;
CM_t=BoomInfo.Aero.Cm;
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
twist1=zeros(1,length(eta1));
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
twist2=zeros(1,length(eta2));

%matrice di rotazione da body a blade
Tj2=[sin(lambda)*cos(pitch)+cos(lambda)*sin(coning)*sin(pitch), -cos(lambda)*cos(pitch)+sin(lambda)*sin(coning)*sin(pitch), -cos(coning)*sin(pitch)
    cos(lambda)*cos(coning), sin(lambda)*cos(coning), +sin(coning)
    sin(lambda)*sin(pitch)-cos(lambda)*sin(coning)*cos(pitch), -cos(lambda)*sin(pitch)-sin(lambda)*sin(coning)*cos(pitch), cos(coning)*cos(pitch)];
psi_ciclo=linspace(0,2*pi,5);
psi_vect=midspan(psi_ciclo);

F_ciclopsi=zeros(3,5);
% M_ciclopsi=[];
for j=1:length(psi_vect)
    psi=psi_vect(j);
T_ciclopsi=[cos(psi), sin(psi), 0
    -sin(psi), cos(psi), 0
    0, 0, 1];

xac1_ciclopsi=T_ciclopsi'*[xac1; 0; 0];
xac2_ciclopsi=T_ciclopsi'*[xac2; 0; 0];


ra1=(xac1_ciclopsi+(Tj1'*[zeros(1,numel(eta1));eta1;zeros(1,numel(eta1))]))';
ra2=(xac2_ciclopsi+(Tj2'*[zeros(1,numel(eta2));eta1;zeros(1,numel(eta2))]))';
%velocity of blade element
omega_m=[omega(1).*ones(numel(eta1),1) omega(2).*ones(numel(eta1),1) omega(3).*ones(numel(eta1),1)];
vel1=u'+cross(omega_m,ra1);
vel2=u'+cross(omega_m,ra2);
%relative velocity of blade in blade frame
w1= Tj1*(-vel1-[0 0 -v_ind_old])';
w2= Tj2*(-vel2-[0 0 -v_ind_old])';
%AoA
AoA1=atan2(w1(3,:),w1(1,:));
AoA2=atan2(w2(3,:),w2(1,:));

net=length(eta1);


CL_naca=interp1(alpha_t, CL_t, AoA1);
CD_naca=interp1(alpha_t, CD_t, AoA1);
CM_naca=interp1(alpha_t, CM_t, AoA1);
F1_i=(span1(2:net+1)-span1(1:net))'.*0.5.*1.225.*c.*(vecnorm(w1([1 3],:)).^2)'.*([-CL_naca.*sin(AoA1)+CD_naca.*cos(AoA1); zeros(size(AoA1)); CL_naca.*cos(AoA1)+CD_naca.*sin(AoA1)])';
CL_naca=interp1(alpha_t, CL_t, AoA2);
CD_naca=interp1(alpha_t, CD_t, AoA2);
CM_naca=interp1(alpha_t, CM_t, AoA2);
F2_i=(span2(2:net+1)-span2(1:net))'.*0.5.*1.225.*c.*(vecnorm(w2([1 3],:)).^2)'.*([-CL_naca.*sin(AoA2)+CD_naca.*cos(AoA2); zeros(size(AoA2)); CL_naca.*cos(AoA2)+CD_naca.*sin(AoA2)])';

F1_i=((Tj1')*F1_i');
F2_i=((Tj2')*F2_i');

F1t=sum(F1_i,2);
F2t=sum(F2_i,2);
F=F1t+F2t;

F_ciclopsi(:,j)=F;
end

%calcolo v_indotta
S=2.28*10^-1;
v_ind_new=0;
for j=1:length(psi_vect)
integral=1/(2*pi)*F_ciclopsi(3,j)/(2*1.225*S*sqrt(u(1)^2+(u(3)-v_ind_old)^2))*(psi_ciclo(j+1)-psi_ciclo(j));
v_ind_new=v_ind_new+integral;
end
vel_ind=v_ind_new;