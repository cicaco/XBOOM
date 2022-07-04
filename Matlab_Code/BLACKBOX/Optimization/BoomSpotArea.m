function A = BoomSpotArea(BoomInfo,Num,D,theta,Chi,varargin)
lb=[3 60 ]; %[Hz gradi m/s]
ub=[12 90 ];
F_dist=@(x) StabSpot(BoomInfo,x,Chi,D,theta);
Xm=[lb(1)+(ub(1)-lb(1))*rand(Num,1) lb(2)+(ub(2)-lb(2))*rand(Num,1)];
S=0;
index=[];
P=cell(Num,1);
P(:)={'NotBack'};
parfor ii=1:Num
    
    Dist(ii)=F_dist(Xm(ii,:)*10);
    if Dist(ii)<5
        S=S+1;
        P(ii)={'Back'};
        index=[index ii];
    end
end
%%
if S>0
X = Xm;
Y = P;
SVMModels = cell(2,1);
classes ={'Back','NotBack'}';
for kk = 1:numel(classes)
    indx = strcmp(Y,classes(kk)); % Create binary classes for each classifier
    SVMModels{kk} = fitcsvm(X,indx,'ClassNames',[false true],'Standardize',true,...
        'KernelFunction','rbf','BoxConstraint',1);
end
%Trovo i nuovi punti da provare all'interno della regione identificata da
%fitcsvm
[x1Grid,x2Grid] = meshgrid(linspace(lb(1),ub(1),100),linspace(lb(2),ub(2),100));
xGrid = [x1Grid(:),x2Grid(:)];
N = size(xGrid,1);
Scores = zeros(N,numel(classes));
tt=1;
cont=1;
XB_Spot=[];
for tt=1:N
    for kk = 1:numel(classes)
        [~,score] = predict(SVMModels{kk},xGrid(tt,:));
        Scores(tt,kk) = score(2); % Second column contains positive-class scores
    end
    if Scores(tt,1)<Scores(tt,2)
        maxScore(tt)=2;
        
        %         if Scores(tt,2)>DIF
        %             maxScore(tt)=2;
        %         else
        %             maxScore(tt)=1;
        %             XB_Spot(cont,:)=xGrid(tt,:);
        %             cont=cont+1;
        %             tt=tt+1;
        %         end
    else
        maxScore(tt)=1;
        XB_Spot(cont,:)=xGrid(tt,:);
        cont=cont+1;
    end
    tt=tt+1;
    
end
try
shp=alphaShape(XB_Spot(:,1),XB_Spot(:,2),'HoleThreshold',10);
catch
    fprintf('ZioPera')
end
        
A=area(shp);
if not(isempty(varargin))
    figure()
        plot(shp)

    hold on
        h(1:2) = gscatter(xGrid(:,1),xGrid(:,2),maxScore);

    h(3:4) = gscatter(X(:,1),X(:,2),Y,'bk','xo');
    scatter(XB_Spot(:,1),XB_Spot(:,2),'om')
    title('{\bf Regione dei lanci corretti e non}');
    xlabel('r0 rad/s');
    ylabel('Phi °');
    legend(h,{'Regione di Lancio',...
        'Alpha','Regione in cui non lanciare','Lanci Sbagliati','Lanci Corretti','observed virginica'},...
        'Location','Northwest');
    
    axis fill
    axis square
    
end
else
    A=0;
end
end

