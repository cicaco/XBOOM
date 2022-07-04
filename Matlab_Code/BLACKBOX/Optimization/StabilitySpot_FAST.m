function [S,Time,Dist,Xm,index] = StabilitySpot_FAST(BoomInfo,lb,ub,N,theta,D,Chi)
%% STABILITYCHECK è una funzione che permette di stimare su N lanci con
% condizioni iniziali randomiche in un certo range quante volte effettivamente il
% boomerang torna indietro
% INPUT
% - BoomInfo: Struct dati del Boomerang
% - lb. Limite inferiore delle condizoni iniziali
% - ub: Limite superiore delle condizoni iniziali
% - N: numero di lanci N
% OUTPUT:
% - S: Percentuale di lanci riusciti
% - Time: Tempo dei lanci  (DIM:Nx1)
% - Dist: Distanza finale dal lanciatore (DIM:Nx1)
% - Xm: Condizioni iniziali degli N lanci (DIM:Nx5)
%%
fprintf('%d simulazioni per il calcolo di stabilità ... \n',N);

tfin=40;
z0= 1.8; % initial altitude
Dist=zeros(N,1);
Time=zeros(N,1);
S=0;
R=norm(BoomInfo.Aero.P_Finish_Dx);
rm=[linspace(lb(1),ub(1),4) linspace(lb(1),ub(1),4) linspace(lb(1),ub(1),4) linspace(lb(1),ub(1),4)]*2*pi;
phim=linspace(lb(2)*pi/180,ub(2)*pi/180,4);
phi_m=[phim(1).*ones(1,4) phim(2).*ones(1,4) phim(3).*ones(1,4) phim(4).*ones(1,4)];
Xm=[rm;phi_m];
index=[];
parfor i=1:16
        
        r0=rm(i);
        phi=phi_m(i);
        Vs=r0*R*(1/Chi-1);

        [quat,ustart] = HandInitial(r0,theta,D,phi,Vs,eye(3),BoomInfo);
        
        options = odeset('Events', @EventsQUAT,'RelTol',1e-4,'AbsTol',1e-6);
        Y0=[quat 0 0 r0  ustart(1) ustart(2) ustart(3) 0 0 z0 ]';
        [TOUT,YOUT_quat] = ode45(@(t,y)EquationOfMotionsQuaternion(t,y,BoomInfo,eye(3)),[0 tfin],Y0,options); %
        Time_i=TOUT(end);
        Dist_i=norm(YOUT_quat(end,11:13));
        [YOUT] = Eul_Quat(YOUT_quat,TOUT);
        if max(vecnorm(YOUT_quat(:,11:13)'))/1.1<=Dist_i
            Dist_i=1000;
        elseif Dist_i<5 
            
            S=S+1;
            i
            index=[index i];
        end
end
Xm=Xm';
figure()
plot(Xm(:,1),Xm(:,2)*180/pi,'*r');
hold on
plot(Xm(index,1),Xm(index,2)*180/pi,'ob');
% shp=alphaShape(Xm(index,1),Xm(index,2));
% plot(shp)
% A=area(shp)
Ind=[1+(index-mod(index,4))/4;mod(index,4)];
% trovo nuovi estremi di 

grid on

% index=[];
% parfor i=1:N
%     r0=Xm(i,1)*2*pi;
%     phi=Xm(i,2)*pi/180;
%     
%     
%     [quat,ustart] = HandInitial(r0,theta,D,phi,Vs,eye(3),BoomInfo);
%     
%     options = odeset('Events', @EventsQUAT,'RelTol',1e-4,'AbsTol',1e-6);
%     Y0=[quat 0 0 r0  ustart(1) ustart(2) ustart(3) 0 0 z0 ]';
%     [TOUT,YOUT_quat] = ode45(@(t,y)EquationOfMotionsQuaternion(t,y,BoomInfo,eye(3)),[0 tfin],Y0,options); %
%     Time_i=TOUT(end);
%     Dist_i=norm(YOUT_quat(end,11:13));
%     
%     if max(vecnorm(YOUT_quat(:,11:13)'))/1.1<=Dist_i
%         Dist_i=1000;
%     elseif Dist_i<5
%         S=S+1;
%         i
%         index=[index i];
%     end
%     Dist(i)=Dist_i;
%     Time(i)=Time_i;
%     
% end
% S=S/N*100;
% fprintf('Stabilità: %.3f \n',S);
% end

