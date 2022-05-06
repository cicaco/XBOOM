function Shape=Boom3DShape(BoomData)
tic
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
Xp_flip=-(-Chord/2.*ones(size(Xp_2d_b))+Xp_2d_b)+Chord/2.*ones(size(Xp_2d_b));
Zp_flip=(Zp_2d_b);
[n,~]=size(Xp_2d_b);
%Clock-wise direction regeneration
Xp_flip=[fliplr(Xp_flip(1:n/2)')';fliplr(Xp_flip(n/2+1:end)')'];
Zp_flip=[fliplr(Zp_flip(1:n/2)')';fliplr(Zp_flip(n/2+1:end)')'];
% 
% figure()
% plot(Xp_2d_b,Zp_2d_b,'k','linewidth',1);
% grid on
% axis equal
% ylabel('$Z_{b}$','fontsize',10,'interpreter','latex')
% xlabel('$X_{b}$','fontsize',10,'interpreter','latex')
% 
% legend({'Naca0012'},'fontsize',8,'interpreter','latex')
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
Y_span_center=[Y_span_center_p -fliplr(Y_span_center_p)];
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
S_sx=[Shape_d(1:6500,:);Shape_v(1:6700,:)];
S_dx=[Shape_d(6501:end,:);Shape_v(6701:end,:)];

shp = alphaShape(S_sx(:,1),S_sx(:,2),S_sx(:,3),1);
[bf_sx, P_sx] = boundaryFacets(shp);
 shp = alphaShape(S_dx(:,1),S_dx(:,2),S_dx(:,3),1);
[bf_dx, P_dx] = boundaryFacets(shp);
nv=length(P_sx);
contri=triangulation([bf_sx ;bf_dx+nv],[P_sx ;P_dx]);
stlwrite(contri, Filename);
toc
end

