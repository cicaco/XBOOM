function [CD]=CD(AoA)
alpha=AoA*180/pi;
if alpha<-180 || alpha>180
    fprintf('error')
else if -180<=alpha && alpha<-170
        CD=0.01;
    else if -170<=alpha && alpha<-90
            CD=1.59/80*(alpha+170)+0.01;
        else if -90<=alpha && alpha<-12
                CD=-1.59/78*(alpha+90)+1.6;
            else if -12<=alpha && alpha<12
                    CD=0.01;
                else if 12<=alpha && alpha<90
                        CD=1.59/78*(alpha-12)+0.01;
                    else if 90<=alpha && alpha<170
                            CD=-1.59/80*(alpha-90)+1.6;
                                else if 170<=alpha && alpha<=180
                                        CD=0.01;
                                    end
                                end
                            end
                        end
                    end
                end
            end
end
end
