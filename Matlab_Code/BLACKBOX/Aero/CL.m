function [CL]=CL(AoA)
%CL compute the CL for an angle of attack range -180/180
%CL=CL(AoA) computes the lift coefficent for a given AoA
%Taken from: Azuma, Akira, et al. "Flight dynamics of the boomerang, part 1:
%fundamental analysis." Journal of guidance, control, and dynamics
27.4 (2004): 545-554.
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

                            
                    
