function [CL]=CL(AoA)
alpha=AoA*180/pi;
if alpha<-180 || alpha>180
    fprintf('error')
else if -180<=alpha && alpha<-170
        CL=0.7/10*(alpha+180);
    else if -170<=alpha && alpha<-130
            CL=0.7;
        else if -130<=alpha && alpha<-60
                CL=-1.7/70*(alpha+130)+0.7;
            else if -60<=alpha && alpha<-12
                    CL=-1;
                else if -12<=alpha && alpha<12
                        CL=2/24*alpha;
                    else if 12<=alpha && alpha<60
                            CL=1;
                        else if 60<=alpha && alpha<130
                                CL=-1.7/70*(alpha-60)+1;
                            else if 130<=alpha && alpha<170
                                    CL=-0.7;
                                else if 170<=alpha && alpha<=180
                                        CL=0.7/10*(alpha-170)-0.7;
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
    end
end
end

                            
                    
