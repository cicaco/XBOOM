function [CM]=CM(AoA)
alpha=AoA*180/pi;
if alpha<-180 || alpha>180
    fprintf('error')
else if -180<=alpha && alpha<-150
        CM=0.4/30*(alpha+180);
    else if -150<=alpha && alpha<-90
            CM=0.4;
        else if -90<=alpha && alpha<0
                CM=-0.4/90*(alpha+90)+0.4;
            else if 0<=alpha && alpha<90
                    CM=-0.4/90*(alpha);
                else if 90<=alpha && alpha<150
                        CM=-0.4;
                    else if 150<=alpha && alpha<=180
                            CM=0.4/30*(alpha-150)-0.4;
                                end
                            end
                        end
                    end
                end
            end
end
end