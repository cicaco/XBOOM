function [S,Time,Dist,Xm] = StabilityCheck(BoomInfo,lb,ub,N)
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
Xm=[lb(1)+(ub(1)-lb(1))*rand(N,1) lb(2)+(ub(2)-lb(2))*rand(N,1) ...
    lb(3)+(ub(3)-lb(3))*rand(N,1) lb(4)+(ub(4)-lb(4))*rand(N,1) ...
    lb(5)+(ub(5)-lb(5))*rand(N,1)];
S=0;
parfor i=1:N 
    r0=Xm(i,1)*2*pi;
    theta=Xm(i,2)*pi/180;
    D=Xm(i,3)*pi/180;
    phi=Xm(i,4)*pi/180;
    Vs=Xm(i,5);
    [quat,ustart] = HandInitial(r0,theta,D,phi,Vs,eye(3),BoomInfo);
    
    options = odeset('Events', @EventsQUAT,'RelTol',1e-4,'AbsTol',1e-6);
    Y0=[quat 0 0 r0  ustart(1) ustart(2) ustart(3) 0 0 z0 ]';
    [TOUT,YOUT_quat] = ode45(@(t,y)EquationOfMotionsQuaternion(t,y,BoomInfo,eye(3)),[0 tfin],Y0,options); %
    Time_i=TOUT(end);
    Dist_i=norm(YOUT_quat(end,11:13));
    
    if max(vecnorm(YOUT_quat(:,11:13)'))/1.1<=Dist_i
        Dist_i=1000;
    elseif Dist_i<5
        S=S+1;
    end
    Dist(i)=Dist_i;
    Time(i)=Time_i;
    
end
S=S/N*100;
fprintf('Stabilità: %.3f \n',S);
end

