function [YOUT] = Eul_Quat(YOUT_quat,TOUT)
euler=[];
for i=1:numel(TOUT)
    euli = quatToEuler(YOUT_quat(i,1:4) );
    euler(i,:)=[euli(2) euli(1) euli(3)];
end
YOUT=[unwrap(euler) YOUT_quat(:,5:end)];
end

