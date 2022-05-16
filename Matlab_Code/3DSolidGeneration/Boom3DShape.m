function [Shape,RBP]=Boom3DShape(BoomData)
tic
% Definisco i dati iniziali, profilo e forma in pianta più le due
% percentuali
Xp_2d_b=BoomData.Profil2D.X;
Zp_2d_b=BoomData.Profil2D.Z;
Chord=BoomData.Profil2D.Chord;
x_0=BoomData.Profil3D.x0;
y_0=BoomData.Profil3D.y0;
z_0=BoomData.Profil3D.z0;
x_f=BoomData.Profil3D.xf;
y_f=BoomData.Profil3D.yf;
z_f=BoomData.Profil3D.zf;
k_1=BoomData.Profil3D.k1;
k_2=BoomData.Profil3D.k2;
k_3=BoomData.Profil3D.k3;
k_4=BoomData.Profil3D.k4;
k_5=BoomData.Profil3D.k5;
k_6=BoomData.Profil3D.k6;
k_7=BoomData.Profil3D.k7;
k_8=BoomData.Profil3D.k8;
Perc_middle=BoomData.Profil3D.Perc_middle;
Perc_center=BoomData.Profil3D.Perc_center;
Filename=BoomData.filename;
% Flippo il profilo in mezzo, prestando sempre attenzione a partire dal
% centro
Xp_flip=-(-Chord/2.*ones(size(Xp_2d_b))+Xp_2d_b)+Chord/2.*ones(size(Xp_2d_b));
Zp_flip=(Zp_2d_b);
[n,~]=size(Xp_2d_b);
%Clock-wise direction regeneration
Xp_flip=[fliplr(Xp_flip(1:n/2)')';fliplr(Xp_flip(n/2+1:end)')'];
Zp_flip=[fliplr(Zp_flip(1:n/2)')';fliplr(Zp_flip(n/2+1:end)')'];

%Creo la pianta 3D date le curve di Bezier
Plant3D=Profile3D(x_0,y_0,z_0,x_f,y_f,z_f,k_1,k_2,k_3,k_4,k_5,k_6,k_7,k_8);

%Trovo il minimo ed il massimo in Y che a regola dovrebbero essere fissi
Y_min_3D=min(Plant3D(2,:));
Y_max_3D=max(Plant3D(2,:));


%%
%%
%Pinta 3D 1-100 ecc
Perc_tip=100-Perc_center-Perc_middle;

% Ordino le Y dal positivo al negativo partendo dalle TIP, MIDDLE, CENTER e
% poi ancora CENTER MIDDLE TIP per le y negative (y_n)
Y_span_center_p=fliplr(Plant3D(2,1:Perc_center-1));
Y_span_middle_p=fliplr(Plant3D(2,Perc_center:Perc_center+Perc_middle-1));
Y_span_tip_p=fliplr(Plant3D(2,Perc_center+Perc_middle:100));
Y_span_center=[Y_span_center_p -fliplr(Y_span_center_p(1:Perc_center-2))];
Y_span_tip_n= -fliplr(Y_span_tip_p);
Y_span_middle_n= -fliplr(Y_span_middle_p);
%create Tip positive
XStart=Xp_2d_b;
ZStart=Zp_2d_b;%zeros(1,n);
ZFinish=Zp_2d_b;
Y_span=Y_span_tip_p;
%Shape scales è la funzione che effettivamente crea la forma 3dimensionale
%dato le X ( che sono sempre costanti) e le Z start e Z finish ed effettua
%una transizione

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
ZFinish=Zp_2d_b;%zeros(1,n);
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
% Unisco dorso e ventre per la triangolazione
Shape_d=[Shape_d_tip_p;Shape_d_middle_p;Shape_d_center;Shape_d_middle_n;Shape_d_tip_n];
Shape_v=[Shape_v_tip_p;Shape_v_middle_p;Shape_v_center;Shape_v_middle_n;Shape_v_tip_n];
Shape=[Shape_d;Shape_v];

S_1=[Shape_d(1:99*66,:);Shape_v(1:99*66,:)];
S_sx=[Shape_d(99*66-66:100*66,:);Shape_v(99*66-66:100*66,:)];
S_2=[Shape_d(100*66-66:101*66,:);Shape_v(100*66-66:101*66,:)];
S_dx=[Shape_d(101*66-66:199*66,:);Shape_v(101*66-66:199*66,:)];

% shp = alphaShape(S_sx(:,1),S_sx(:,2),S_sx(:,3),1);
% [bf_sx, P_sx] = boundaryFacets(shp);
%  shp = alphaShape(S_dx(:,1),S_dx(:,2),S_dx(:,3),1);
% [bf_dx, P_dx] = boundaryFacets(shp);
% nv=length(P_sx);
warning('off','all')
shp = alphaShape(S_1(:,1),S_1(:,2),S_1(:,3),1);
[bf_1, P_1] = boundaryFacets(shp);
 shp = alphaShape(S_sx(:,1),S_sx(:,2),S_sx(:,3),1);
[bf_sx, P_sx] = boundaryFacets(shp);
nv_1=length(P_1);
shp = alphaShape(S_2(:,1),S_2(:,2),S_2(:,3),1);
[bf_2, P_2] = boundaryFacets(shp);
nv_sx=length(P_sx);
 shp = alphaShape(S_dx(:,1),S_dx(:,2),S_dx(:,3),1);
[bf_dx, P_dx] = boundaryFacets(shp);
nv_2=length(P_2);
warning('on','all')

contri=triangulation([bf_1;bf_sx+nv_1;bf_2+nv_sx+nv_1;bf_dx+nv_2+nv_sx+nv_1],[P_1;P_sx ;P_2;P_dx]);
% shp = alphaShape(Shape(:,1),Shape(:,2),Shape(:,3),1);
% [bf, P] = boundaryFacets(shp);
stlwrite(contri, Filename);
%Viene chiamata la funzione del tipo
RBP=RigidBodyParams(contri);
%disp(RBP)
%VisualizeLocalFrame(contri)

end

