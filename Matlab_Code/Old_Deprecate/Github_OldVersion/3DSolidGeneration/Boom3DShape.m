function [S,RBP,Plant3D]=Boom3DShape(BoomData)
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
fprintf('Strating Creating Geometry... \n');

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

[Shape_d_tip_p,Shape_v_tip_p,Shape_tip_p] = ShapeScales(XStart,ZFinish,ZStart,Y_span,Plant3D);

%create Middle positive
XStart=Xp_2d_b;
ZStart=Zp_2d_b;
ZFinish=Zp_2d_b;
Y_span=Y_span_middle_p;
[Shape_d_middle_p,Shape_v_middle_p,Shape_middle_p] = ShapeScales(XStart,ZFinish,ZStart,Y_span,Plant3D);

%create Center positive
XStart=Xp_2d_b;
ZStart=Zp_2d_b;
ZFinish=Zp_flip;
Y_span=Y_span_center;
[Shape_d_center,Shape_v_center,Shape_center] = ShapeScales(XStart,ZFinish,ZStart,Y_span,Plant3D);
%create Tip positive
XStart=Xp_2d_b;
ZFinish=Zp_2d_b;%zeros(1,n);
ZStart=Zp_flip;
Y_span=Y_span_tip_n;
[Shape_d_tip_n,Shape_v_tip_n,Shape_tip_n]= ShapeScales(XStart,ZFinish,ZStart,Y_span,Plant3D);

%create Middle negative
XStart=Xp_2d_b;
ZStart=Zp_flip;
ZFinish=Zp_flip;
Y_span=Y_span_middle_n;
[Shape_d_middle_n,Shape_v_middle_n,Shape_middle_n] = ShapeScales(XStart,ZFinish,ZStart,Y_span,Plant3D);

% Inizio a posizionare i profili che dovranno essere scalati correttamente
% Unisco dorso e ventre per la triangolazione
Shape_d=[Shape_d_tip_p;Shape_d_middle_p;Shape_d_center;Shape_d_middle_n;Shape_d_tip_n];
Shape_v=[Shape_v_tip_p;Shape_v_middle_p;Shape_v_center;Shape_v_middle_n;Shape_v_tip_n];
Shape=[Shape_tip_p;Shape_middle_p;Shape_center;Shape_middle_n;Shape_tip_n];


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
    while cont<=size(Shape_del,1) && Shape_del(cont,4)==j 
        Profile=[Profile; Shape_del(cont,:)];
        num=size(Profile,1);
        cont=cont+1;

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
% ora devo creare la triangolazione che ho tutti i punti unici: deve andare
% sempre dal profilo avente y maggiore a minore 
cont =1;
Prec=1;
T=[];
n=[];
for j=1:198
    cont=Prec;
    P_up=[];
    while cont<=size(S,1) && S(cont,4)==j
        P_up=[P_up; S(cont,:) cont];
        num=size(P_up,1);
        cont=cont+1;
    end
    Prec=cont;
    
    P_down=[];
    while cont<=size(S,1) && S(cont,4)==j+1
        P_down=[P_down; S(cont,:) cont];
        num=size(P_down,1);
        cont=cont+1;
    end
    
    n_up=size(P_up,1);
    n_down=size(P_down,1);
    n=[n; j n_up n_down];
    %Tip positiva
    if n_up==1 % corda all estremità
        for kk=1:n_down-1
            T=[T; P_up(1,5) P_down(kk,5) P_down(kk+1,5) j];
        end
        T=[T; P_up(1,5) P_down(n_down,5) P_down(1,5)  j];
    else
        if n_down==1 % corda all estremità
            for kk=1:n_up-1
                T=[T;  P_up(kk,5) P_down(1,5) P_up(kk+1,5) j];
            end
            T=[T;  P_up(n_up,5) P_down(1,5) P_up(1,5) j];
        else
            if n_down==n_up
                for ii=1:n_up-1
                    T=[T;  P_up(ii,5) P_down(ii,5) P_up(ii+1,5) j;...
                        P_up(ii+1,5)  P_down(ii,5)  P_down(ii+1,5) j];
                end
                               T=[T;  P_up(n_up,5) P_down(n_up,5) P_up(1,5) j;...
                                   P_up(1,5) P_down(n_up,5) P_down(1,5) j];
            else
                if n_up<n_down
                    for ii=1:n_up-1
                    T=[T;  P_up(ii,5) P_down(ii+1,5) P_up(ii+1,5) j;...
                        P_up(ii+1,5)  P_down(ii+1,5)  P_down(ii+2,5) j];
                    end
                     T=[T;  P_up(n_up,5) P_down(n_down,5) P_down(1,5) j;...
                                   P_up(n_up,5) P_down(1,5) P_up(1,5) j;...
                                   P_up(1,5) P_down(1,5) P_down(2,5) j;];
                else
                    for ii=1:n_down-1
                    T=[T;  P_up(ii,5) P_down(ii,5) P_up(ii+1,5) j;...
                        P_up(ii+1,5)  P_down(ii,5)  P_down(ii+1,5) j];
                    end
                         T=[T;  P_up(n_up-1,5) P_down(n_down,5) P_up(n_up,5) j;...
                                   P_up(n_up,5) P_down(n_down,5) P_down(1,5) j;...
                                   P_up(n_up,5) P_down(1,5) P_up(1,5) j;];
                    
                end
            end
        end
    end
end
%Rotation of Cordinattes
P_xyz=S(:,1:3);
TR = triangulation(T(:,1:3),P_xyz);

RBP=RigidBodyParams(TR);
%Change the refernce system to the body's one

G=[RBP.centroid(1) 0 0];
R=[-1 0 0; 0 1 0; 0 0 1];
Trasl=G';
% size(P_xyz')
% size(Trasl.*ones(size(P_xyz')))

P_xyz_body=(R*(P_xyz'-Trasl.*ones(size(P_xyz'))))';
TR = triangulation(T(:,1:3),P_xyz_body);
stlwrite(TR,Filename,'text')

%VisualizeLocalFrame(TR)
end

