syms sigma beta theta pi2

R1=[cos(theta) 0 sin(theta)
    0 1 0
    -sin(theta) 0 cos(theta)];

R2=[1 0 0
    0 cos(beta) -sin(beta) 
    0 +sin(beta) cos(beta)];
R3=[cos(sigma) -sin(sigma) 0
    sin(sigma) cos(sigma) 0
    0 0 1];
R=(R3*R2*R1); % TI porta dal sistema 