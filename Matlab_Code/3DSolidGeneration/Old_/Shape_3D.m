clear all
close all
Profile2D=importdata('Naca0012.dat');
Xp_2d_b=[Profile2D.data(2:67,1) ; fliplr(Profile2D.data(68:end,1)')'];
Zp_2d_b=[Profile2D.data(2:67,2) ; fliplr(Profile2D.data(68:end,2)')'];
Chord=1;
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
x_0=0;
y_0=0;
z_0=0;
x_f=1;
y_f=0;
z_f=0;
k_1= 1.0; % origin point tangent ,y
k_2= 2.0; %mid point, trailing edge x
k_3= 4; %mid point  y
k_4=4.5; %Tip point tangent sx, x
k_5=6; %Tip point  y
k_6=5.0; %Tip point  x
k_7=5.5; %Tip point tangent dx, x
k_8=3.0; %mid point leading edge, x
Plant3D=Profile3D(x_0,y_0,z_0,x_f,y_f,z_f,k_1,k_2,k_3,k_4,k_5,k_6,k_7,k_8);
plot(Plant3D(1,:),Plant3D(2,:),'*r');
%Try to compute in a 3D space
Y_min_3D=min(Plant3D(2,:));
Y_max_3D=max(Plant3D(2,:));
%%
%Pinta 3D 1-100 ecc
Perc_center=10;
Perc_middle=70;
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
k = boundary(Shape(:,1),Shape(:,2),Shape(:,3),1);
tri=delaunay(Shape(:,1),Shape(:,2));
figure(5)
trisurf(k,Shape(:,1),Shape(:,2),Shape(:,3),'Facecolor','red','FaceAlpha',0.1,'EdgeColor','none')
axis equal
figure(8)
shp = alphaShape(S_sx(:,1),S_sx(:,2),S_sx(:,3),1);
[bf_sx, P_sx] = boundaryFacets(shp);
 shp = alphaShape(S_dx(:,1),S_dx(:,2),S_dx(:,3),1);
[bf_dx, P_dx] = boundaryFacets(shp);
nv=length(P_sx);
contri=triangulation([bf_sx ;bf_dx+nv],[P_sx ;P_dx]);
 stlwrite(contri, 'Boom.stl');
%size(tri)
%stlwrite(triangulation(T,P),'foo.stl')
h=plot(shp);
h.FaceColor = 'b';
%h.FaceAlpha=0.1;
h.EdgeColor='none';
axis equal
%Shape=Shape_d_center;
figure(4)
plot3(Shape_d(:,1),Shape_d(:,2),Shape_d(:,3),'*b');
grid on
axis equal
% x=Shape_d(:,1)';
% y=Shape_d(:,2)';
% z=Shape_d(:,3)';
% 
% [X,Y] = meshgrid(x, y);
% Z = griddata(x,y,z,X,Y);
% figure(5)
% surf(X, Y, Z);
% grid on
% set(gca, 'ZLim',[0 0.1])
% axis equal
% shading interp