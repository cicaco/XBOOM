syms psi theta phi

R1=[1 0 0
    0 cos(phi) -sin(phi)
    0 sin(phi) cos(phi)];

R2=[cos(theta) 0 sin(theta)
    0 1 0 
    -sin(theta) 0 cos(theta)];
R3=[cos(psi) -sin(psi) 0
    sin(psi) cos(psi) 0
    0 0 1];
R=(R3*R2*R1); % TI porta dal sistema 
% subs(R,[0 0 0])
% 
% S321_I=[cos(psi)*cos(theta) -sin(psi) 0
%         sin(psi)*cos(theta) cos(psi) 0
%         -sin(theta) 0 1];
    
    
%     syms dpsi dtheta dphi
% om=R'*S321_I*[dphi;dtheta; dpsi;]
% p=om(1)
% q=om(2)
% r=om(3)