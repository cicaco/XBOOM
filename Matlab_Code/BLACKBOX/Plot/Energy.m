function  Energy(TOUT,YOUT,BoomInfo)
Ecin=[];
E_pot=[];
Inerzia=BoomInfo.Mecc.I_rho;

M=BoomInfo.Mecc.m;
for i=1:numel(TOUT)
    p=YOUT(i,4);
q=YOUT(i,5);
r=YOUT(i,6);
ux=YOUT(i,7);
uy=YOUT(i,8);
uz=YOUT(i,9);

Ecin(i)=1/2*([p q r]*Inerzia*[p;q;r]+[ux uy uz]*M*[ux;uy;uz]);
E_pot(i)=M*9.81*YOUT(i,12);
V(i)=norm([ux uy uz]);
Om(i)=norm([p q r]);
end
if max(diff(Ecin+E_pot))>10^-3
    fprintf('Il boomerang sta creando energia IMPOSSIBILE \n');
    fprintf('%.4f \n',max(diff(Ecin+E_pot)));
end
linecolors={'r' 'y' 'c' 'g' 'b' 'k'};
[handles]=plotNy(TOUT(:),YOUT(:,6),1,...
    TOUT(:),YOUT(:,12),2,...
    TOUT(:),Ecin',3,...
    TOUT(:),E_pot',3,...
    TOUT(:),E_pot'+Ecin',3,...
    TOUT(:),V',4,...
    TOUT(:),Om',1,...
    'YAxisLabels',{ 'Angular Rate [rad/s]' 'Position [m]' 'Energy [J]' 'Velocità [m/s]'},...
    'Linewidth',1,...
    'XLim',[0,TOUT(end)],...
    'XAxisLabel','time[s]',...
    'TitleStr','Bilancio Energetico',...
    'FontSize',10,...
    'LegendString',{ 'r' 'z' 'E. Cinetica' 'E. Potenziale' 'E. Totale' 'Modulo V' 'Modulo Om'});
grid on

end

