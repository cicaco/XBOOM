function CheckBoomInfo(BoomInfo,varargin)
%CHECKBOOMINFO è una funzione che permette di verificare che tutti i dati
%inseriti siano corretti:
%INPUT:
%-  BoomInfo: struct
%OPZIONI:
%-  'Plot': viene plottato il profilo dx e sx ed i centri aerodinamici, il profilo dx è corretto se il
%   bordo di attacco punto verso sinistra nella figura. Se si vuole spostare 
%   il centro aerodinamico, traslare verso il basso o l alto le coordinate Z. 
I=zeros(1,36);
C_fig=0;

nVarargs = length(varargin);
i=1;
while i<=nVarargs
    switch varargin{i}
        case 'Plot'
            C_fig=1;
        otherwise
            error('Verificare di aver inserito le opzioni corrette ')
    end
    i=i+1;
end
%% Aero
I(1)=isnan(sum(BoomInfo.Aero.alpha_cd));
I(2)=isnan(sum(BoomInfo.Aero.alpha_cd));
I(3)=isnan(sum(BoomInfo.Aero.alpha_cm));
I(4)=isnan(sum(BoomInfo.Aero.Cd));
I(5)=isnan(sum(BoomInfo.Aero.Cl));
I(6)=isnan(sum(BoomInfo.Aero.Cm));
I(7)=isnan(sum(BoomInfo.Aero.P_Finish_Dx));
I(8)=isnan(sum(BoomInfo.Aero.P_Finish_Sx));
I(9)=isnan(sum(BoomInfo.Aero.P_origin_Dx));
I(10)=isnan(sum(BoomInfo.Aero.P_origin_Sx));
I(11)=isnan(sum(BoomInfo.Aero.P_Start_Dx));
I(12)=isnan(sum(BoomInfo.Aero.P_Start_Sx));
I(13)=isnan(sum(BoomInfo.Aero.Start_Dx));
I(14)=isnan(sum(BoomInfo.Aero.Start_Sx)); 
%% Pianta
I(15)=isnan(BoomInfo.Pianta.l);
I(16)=isnan(BoomInfo.Pianta.freccia);
I(17)=isnan(BoomInfo.Pianta.diedro);
I(18)=isnan(BoomInfo.Pianta.pitch);
%% Geom3D
I(19)=isnan(BoomInfo.Geom3D.p_c);
I(20)=isnan(BoomInfo.Geom3D.num);
I(21)=isnan(BoomInfo.Geom3D.PARA);
I(22)=isnan(sum(BoomInfo.Geom3D.Fr_i));
I(23)=isnan(sum(BoomInfo.Geom3D.Di_i));
I(24)=isnan(sum(sum(BoomInfo.Geom3D.Pi_i)));
I(25)=isnan(sum(sum(BoomInfo.Geom3D.Profile)));
I(26)=isnan(sum(sum(BoomInfo.Geom3D.C_aer)));
%% Profile
I(27)=isnan(BoomInfo.Profile.Chord);
I(28)=isnan(sum(BoomInfo.Profile.Xp_dx));
I(29)=isnan(sum(BoomInfo.Profile.Xp_sx));
I(30)=isnan(sum(BoomInfo.Profile.Zp_dx));
I(31)=isnan(sum(BoomInfo.Profile.Zp_sx));
%% Mecc
I(32)=isnan(BoomInfo.Mecc.V);
I(33)=isnan(BoomInfo.Mecc.m);
I(34)=isnan(sum(sum(BoomInfo.Mecc.I)));
I(35)=isnan(sum(sum(BoomInfo.Mecc.I_rho)));
I(36)=isnan(sum(BoomInfo.Mecc.CG));
if sum(I)==1
    error('Dato in BoomInfo con NaN \n');
end
if C_fig==1
    figure()
    plot(BoomInfo.Profile.Xp_dx,BoomInfo.Profile.Zp_dx,'r','linewidth',1.2);
    hold on
    plot(BoomInfo.Profile.Xp_sx,BoomInfo.Profile.Zp_sx,'b','linewidth',1.2);
    plot(-BoomInfo.Profile.Chord/4,0,'*r','linewidth',1.2);
    plot(-3*BoomInfo.Profile.Chord/4,0,'*b','linewidth',1.2);
    axis equal
    grid on
    set(gca,'Xdir','reverse')
    legend('Dx','Sx','$C_{aer}$ Dx','$C_{aer}$ Sx','fontsize',10,'interpreter','latex')
    xlabel('X','fontsize',11,'interpreter','latex');
    set(gca,'TickLabelInterpreter','latex')
    ylabel('Y','fontsize',11,'interpreter','latex');
    figure()
    plot(BoomInfo.Aero.alpha_cl,BoomInfo.Aero.Cl,'--r','linewidth',1.2);
    hold on
    plot(BoomInfo.Aero.alpha_cd,BoomInfo.Aero.Cd,'.-b','linewidth',1.2);
    plot(BoomInfo.Aero.alpha_cm,BoomInfo.Aero.Cm,'k','linewidth',1.2);
    grid on
    legend('CL','CD','CM','fontsize',10,'interpreter','latex')
    ylabel('Coefficienti []','fontsize',11,'interpreter','latex');
    set(gca,'TickLabelInterpreter','latex')
    xlabel('$\alpha$ [Gradi]','fontsize',11,'interpreter','latex');
end
%% Check sull'angolo di attacco
if max(BoomInfo.Aero.alpha_cd)<10 && max(BoomInfo.Aero.alpha_cm)<10 && max(BoomInfo.Aero.alpha_cl)<10
    error('Gli angoli di attacco in BoomInfo.Aero.alpha_** vanno inseriti in Gradi');
end
%% Check sui profili
if norm(BoomInfo.Profile.Xp_dx-BoomInfo.Profile.Xp_sx)>=BoomInfo.Profile.Chord
    fprintf('Il profilo Dx e Sx non viene percorso allo stesso modo \n');
    error('L ERRORE può portare a malfunzionamenti: Correggere BoomInfo.Profile.Xp_sx o BoomInfo.Profile.Xp_dx');
end
if max(BoomInfo.Profile.Xp_dx)>=BoomInfo.Profile.Chord/2
    fprintf('Il profilo non è stato inserito correttamente:\nIl bordo di attacco deve trovarsi nell origine ed il bordo di uscita a -Corda\n');
    error('L ERRORE può portare a malfunzionamenti: Correggere BoomInfo.Profile.Xp_dx e BoomInfo.Profile.Xp_sx');
end
