function [C_aer] = AerCenter(P_fin,Chord,F)
    C_start=[P_fin(1,1) P_fin(2,1)];
    C_fin=[P_fin(1,end) P_fin(2,end)];
    n_c=(C_fin-C_start)/norm(C_fin-C_start);
    C_aer=[C_start'; 0]+[ F.*Chord.*n_c' ; 0];

end

