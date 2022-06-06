clear all
close all
addpath(genpath('Inerzia'));

%Profilo 
Profile2D=importdata('Naca0012.dat');

%Sistemo il plot in senso orario partendo da 0,0
Xp_2d_b=[Profile2D.data(2:67,1) ; fliplr(Profile2D.data(68:end,1)')'];
Zp_2d_b=[Profile2D.data(2:67,2) ; fliplr(Profile2D.data(68:end,2)')'];
Chord=0.1;
%Forma 3D
x_0=0; %Corda dimensione
y_0=0;
z_0=0;
x_f=0.1;
y_f=0;
z_f=0;
k_1=0.05; % origin point tangent ,y
k_2= 0.1; %mid point, trailing edge x
k_3= 0.15; %mid point  y
k_4=0.3; %Tip point tangent sx, x
k_5=0.3; %Tip point  y
k_6=0.35; %Tip point  x
k_7=0.4; %Tip point tangent dx, x
k_8=0.25; %mid point leading edge, x

Perc_center=10; %da che punto iniziare a flippare il profilo
Perc_middle=70; 
Filename='Boom_Naca0012.stl';

BoomData.Profil2D.X=Xp_2d_b;
BoomData.Profil2D.Z=Zp_2d_b;
BoomData.Profil2D.Chord=Chord;
BoomData.Profil3D.x0=x_0;
BoomData.Profil3D.y0=y_0;
BoomData.Profil3D.z0=x_0;
BoomData.Profil3D.xf=x_f;
BoomData.Profil3D.yf=y_f;
BoomData.Profil3D.zf=z_f;
BoomData.Profil3D.k1=k_1;
BoomData.Profil3D.k2=k_2;
BoomData.Profil3D.k3=k_3;
BoomData.Profil3D.k4=k_4;
BoomData.Profil3D.k5=k_5;
BoomData.Profil3D.k6=k_6;
BoomData.Profil3D.k7=k_7;
BoomData.Profil3D.k8=k_8;
BoomData.Profil3D.Perc_center=Perc_center;
BoomData.Profil3D.Perc_middle=Perc_middle;
BoomData.filename=Filename;


Xp_flip=-(-Chord/2.*ones(size(Xp_2d_b))+Xp_2d_b)+Chord/2.*ones(size(Xp_2d_b));
Zp_flip=(Zp_2d_b);
[n,~]=size(Xp_2d_b);
%Clock-wise direction regeneration
Xp_flip=[fliplr(Xp_flip(1:n/2)')';fliplr(Xp_flip(n/2+1:end)')'];
Zp_flip=[fliplr(Zp_flip(1:n/2)')';fliplr(Zp_flip(n/2+1:end)')'];

figure()
plot(Xp_2d_b,Zp_2d_b,'k','linewidth',1);
grid on
axis equal
ylabel('$Z_{b}$','fontsize',10,'interpreter','latex')
xlabel('$X_{b}$','fontsize',10,'interpreter','latex')

legend({'Naca0012'},'fontsize',8,'interpreter','latex')
% Scelgo una corda unitaria

Plant3D=Profile3D(x_0,y_0,z_0,x_f,y_f,z_f,k_1,k_2,k_3,k_4,k_5,k_6,k_7,k_8);
plot(Plant3D(1,:),Plant3D(2,:),'*r');
%Try to compute in a 3D space
Y_min_3D=min(Plant3D(2,:));
Y_max_3D=max(Plant3D(2,:));


%%
%%
%Pinta 3D 1-100 ecc
Perc_tip=100-Perc_center-Perc_middle;
% if Perc_tip<0
%     error('Errore, percentuale totale minore di 0');
% end
% Y_span_tip_n=fliplr(linspace(Y_min_3D,(100-Perc_tip-1)/100*Y_min_3D,Perc_tip));
% Y_span_middle_n=fliplr(linspace((100-Perc_tip)/100*Y_min_3D,(100-Perc_tip-Perc_middle-1)/100*Y_min_3D,Perc_middle));
% Y_span_center_n=fliplr(linspace((100-Perc_tip-Perc_middle)/100*Y_min_3D,(100-99)/100*Y_min_3D,Perc_center-1));
% Y_span_tip_p=-fliplr(Y_span_tip_n);
% Y_span_middle_p=-fliplr(Y_span_middle_n);
% Y_span_center_p=-fliplr(Y_span_center_n);
% 
Y_span_center_p=fliplr(Plant3D(2,1:Perc_center-1));
Y_span_middle_p=fliplr(Plant3D(2,Perc_center:Perc_center+Perc_middle-1));
Y_span_tip_p=fliplr(Plant3D(2,Perc_center+Perc_middle:100));
Y_span_center=[Y_span_center_p -fliplr(Y_span_center_p(1:Perc_center-2))];
%Y_span_center=[Y_span_center_p -fliplr(Y_span_center_p)];
Y_span_tip_n= -fliplr(Y_span_tip_p);
Y_span_middle_n= -fliplr(Y_span_middle_p);
%create Tip positive
XStart=Xp_2d_b;
ZStart=zeros(1,n);
ZFinish=Zp_2d_b;
Y_span=Y_span_tip_p;
[Shape_d_tip_p,Shape_v_tip_p] = ShapeScales(XStart,ZFinish,ZStart,Y_span,Plant3D);

%create Middle positive
XStart=Xp_2d_b;
ZStart=Zp_2d_b;
ZFinish=Zp_2d_b;
Y_span=Y_span_middle_p;
[Shape_d_middle_p,Shape_v_middle_p] = ShapeScales(XStart,ZFinish,ZStart,Y_span,Plant3D);

%create Center positive
XStart=Xp_2d_b;
ZStart=Zp_2d_b;
ZFinish=Zp_flip;
Y_span=Y_span_center;
[Shape_d_center,Shape_v_center] = ShapeScales(XStart,ZFinish,ZStart,Y_span,Plant3D);
%create Tip positive
XStart=Xp_2d_b;
ZFinish=zeros(1,n);
ZStart=Zp_flip;
Y_span=Y_span_tip_n;
[Shape_d_tip_n,Shape_v_tip_n]= ShapeScales(XStart,ZFinish,ZStart,Y_span,Plant3D);

%create Middle negative
XStart=Xp_2d_b;
ZStart=Zp_flip;
ZFinish=Zp_flip;
Y_span=Y_span_middle_n;
[Shape_d_middle_n,Shape_v_middle_n] = ShapeScales(XStart,ZFinish,ZStart,Y_span,Plant3D);

% Inizio a posizionare i profili che dovranno essere scalati correttamente

Shape_d=[Shape_d_tip_p;Shape_d_middle_p;Shape_d_center;Shape_d_middle_n;Shape_d_tip_n];
Shape_v=[Shape_v_tip_p;Shape_v_middle_p;Shape_v_center;Shape_v_middle_n;Shape_v_tip_n];
Shape=[Shape_d;Shape_v];
S_sx=[Shape_d(1:100*66,:);Shape_v(1:100*66,:)];
% S_1=[Shape_d(1:98*66,:);Shape_v(1:98*66,:)];
% S_sx=[Shape_d(98*66-66:100*66,:);Shape_v(99*66-66:100*66,:)];
% S_2=[Shape_d(100*66-66:102*66,:);Shape_v(100*66-66:102*66,:)];
% S_dx=[Shape_d(102*66-66:end,:);Shape_v(102*66-66:end,:)];


% shp = alphaShape(S_1(:,1),S_1(:,2),S_1(:,3),1);
% [bf_1, P_1] = boundaryFacets(shp);
%  shp = alphaShape(S_sx(:,1),S_sx(:,2),S_sx(:,3),1);
% [bf_sx, P_sx] = boundaryFacets(shp);
% nv_1=length(P_1);
% shp = alphaShape(S_2(:,1),S_2(:,2),S_2(:,3),1);
% [bf_2, P_2] = boundaryFacets(shp);
% nv_sx=length(P_sx);
%  shp = alphaShape(S_dx(:,1),S_dx(:,2),S_dx(:,3),1);
% [bf_dx, P_dx] = boundaryFacets(shp);
% nv_2=length(P_2);
%contri=triangulation([bf_1;bf_sx+nv_1;bf_2+nv_sx+nv_1;bf_dx+nv_2+nv_sx+nv_1],[P_1;P_sx ;P_2;P_dx]);
S_sx=[Shape_d(1:6600,:);Shape_v(1:6600,:)];
S_dx=[Shape_d(6534:end,:);Shape_v(6534:end,:)];
shp = alphaShape(S_sx(:,1),S_sx(:,2),S_sx(:,3),1);
[bf_sx, P_sx] = boundaryFacets(shp);
shp = alphaShape(S_dx(:,1),S_dx(:,2),S_dx(:,3),1);
[bf_dx, P_dx] = boundaryFacets(shp);
nv=length(P_sx);
contri=triangulation([bf_sx ;bf_dx+nv],[P_sx ;P_dx]);

stlwrite(contri, Filename);


%%

% Provo a creare un meshgrid automatico
% Shape d da ymax a ymin sono dorso con 66 componenti per un totale di 199
% "linee"
Xd=reshape(Shape_d(:,1),[66 199]);
Yd=reshape(Shape_d(:,2),[66 199]);
Zd=reshape(Shape_d(:,3),[66 199]);
Shape2=[];
for i=1:199
    Shape2=[Shape2;Shape_d(66*(i-1)+1:66*i,:)];
    Shape2=[Shape2;Shape_v(66*(i-1)+1:66*i,:)];
end

Xd=reshape(Shape2(:,1),[132 199]);
Yd=reshape(Shape2(:,2),[132 199]);
Zd=reshape(Shape2(:,3),[132 199]);
figure()
surf(Xd,Yd,Zd,'LineStyle','none')
axis equal
%% Provo a creare la triangolazione 
% Dal Shape2 ogni sezione ha 132 punti, il primo è il dorso e devo
% connetterla con la sezione successiva e parto dal dorso, su goni sezione
% parto dal bordo di attacco , primo triangolo con due vertici sulla prima
% sezione e poi due sulla successiva e una no


%%
T=[];
T_delete=[];
cont=1;
% problemi da risolvere:
%- Eliminare i triangoli con punti in comune
%- Mettere normale uscente sempre
Shape_del=zeros(size(Shape2,1),4);
Shape_del(:,1:3)=Shape2;%zeros(numel(Shape2),1)];
%Elimino tutti i punti doppi per ogni profilo 
for j=1:199
    
for i=1:132
    Shape_del(132*(j-1)+i,4)=j;
end
end

%%
Shape_def=Shape_del;
cont=1;
S=[];
for j=1:199
    Profile=[];
    Inc=[];
    while cont<=size(Shape_del,1) && Shape_del(cont,4)==j 
        Profile=[Profile; Shape_del(cont,:)];
        num=size(Profile,1);
        cont=cont+1;
        Inc=[Inc;cont];
    end
    
    i=1;
   while i<=num
        P1=Profile(i,1:3);
        index=[];
        for k=1:num
            P2=Profile(k,1:3);
            if i~=k
                
                if norm(P1-P2)==0 %&& Shape_del(132*(j-1)+i
                    index=[index k];
                end
            end
        end
         
         Profile(index,:)=[];
         num=size(Profile,1);
         i=i+1;
   end
    S=[S;Profile];
end
%%
for j=1:198
    
for i=1:132
    prec=size(T,1);
     if i==132
         T=[T; 132*(j-1)+1 132*(j-1)+i 132*(j-1)+i+1 j/1000 ; 132*(j-1)+i+1 132*(j-1)+i  132*(j)+i j/1000 ];
     else
         T=[T;132*(j-1)+i 132*(j)+i 132*(j-1)+i+1  j;132*(j-1)+i+1 132*(j)+i 132*(j)+i+1 j];
     end
     succ=size(T,1);
     index=[];
     for k=prec+1:succ
         P1=Shape2(T(k,1),:);
         P2=Shape2(T(k,2),:);
         P3=Shape2(T(k,3),:);
         
         if norm(P1-P2)==0 || norm(P1-P3)==0 || norm(P2-P3)==0
            index=[index k];
            
         end
            
     end
      T_delete=[T_delete;T(index,:)];
      T(index,:)=[];
end
end
j=0;
TR = triangulation(T(:,1:3),Shape2);
%%
TR = triangulation(T(1:130,1:3),1000.*Shape2);

T1=T(264*j+1:264*(j+1),1:3);
%TR = triangulation(T(:,1:3),Shape2);
figure()
stlwrite(TR,'tritext.stl','text')
triplot(TR)
RBP=RigidBodyParams(TR);
figure()
P2=Shape2(j*132+1:(j+2)*132,:);
for i=1:264
plot3(P2(i,1),P2(i,2),P2(i,3),'*r');
hold on
text(P2(i,1),P2(i,2),P2(i,3),string(i));
end

