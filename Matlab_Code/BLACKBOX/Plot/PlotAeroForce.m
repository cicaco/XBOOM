function PlotAeroForce(YOUT,TOUT,BoomInfo,varargin)
nVarargs = length(varargin);
i=1;
C_body=0;
TIT='Sistema di riferimento Body';
while i<=nVarargs
    switch varargin{i}
        case 'Global_fFrame'
            C_body=1;
            TIT='Sistema di riferimento globale';
            
        otherwise
            fprintf('Opzioni sbagliate');
    end
    i=i+1;
end
ux=YOUT(:,7);
uy=YOUT(:,8);
uz=YOUT(:,9);
p=YOUT(:,4);
q=YOUT(:,5);
r=YOUT(:,6);
theta=YOUT(:,1);
phi=YOUT(:,2);
psi=YOUT(:,3);
num=numel(p);
F=zeros(3,num);
M=zeros(3,num);
R=@(theta0,psi0,phi0) [cos(theta0)*cos(psi0), cos(theta0)*sin(psi0), -sin(theta0)
    -cos(phi0)*sin(psi0)+sin(phi0)*sin(theta0)*cos(psi0), cos(phi0)*cos(psi0)+sin(phi0)*sin(theta0)*sin(psi0), sin(phi0)*cos(theta0)
    sin(phi0)*sin(psi0)+cos(phi0)*sin(theta0)*cos(psi0), -sin(phi0)*cos(psi0)+cos(phi0)*sin(theta0)*sin(psi0), cos(phi0)*cos(theta0)];
% BoomInfo.Mecc.I_rho = 3*BoomInfo.Mecc.I_rho;

for i=1:num
    [F(:,i),M(:,i)]=AeroDynamics([ux(i);uy(i);uz(i)],[p(i);q(i);r(i)],BoomInfo);
    if C_body==1
        
T0=[cos(theta(i))*cos(psi(i)), cos(theta(i))*sin(psi(i)), -sin(theta(i))
    -cos(phi(i))*sin(psi(i))+sin(phi(i))*sin(theta(i))*cos(psi(i)), cos(phi(i))*cos(psi(i))+sin(phi(i))*sin(theta(i))*sin(psi(i)), sin(phi(i))*cos(theta(i))
    sin(phi(i))*sin(psi(i))+cos(phi(i))*sin(theta(i))*cos(psi(i)), -sin(phi(i))*cos(psi(i))+cos(phi(i))*sin(theta(i))*sin(psi(i)), cos(phi(i))*cos(theta(i))];

        F(:,i)=T0'*F(:,i);
        M(:,i)=T0'*M(:,i);
    end
end

figure()
subplot(3,2,1)
plot(TOUT(:),F(1,:)','r','linewidth',1);
legend('Fx');
grid on

subplot(3,2,2)
plot(TOUT(:),M(1,:)','r','linewidth',1);
legend('Mx');
grid on

subplot(3,2,3)
plot(TOUT(:),F(2,:)','b','linewidth',1);
legend('Fy');
grid on

subplot(3,2,4)
plot(TOUT(:),M(2,:)','b','linewidth',1);
legend('My');
grid on

subplot(3,2,5)
plot(TOUT(:),F(3,:)','g','linewidth',1);
legend('Fz');
grid on

subplot(3,2,6)
plot(TOUT(:),M(3,:)','g','linewidth',1);
legend('Mz');
sgtitle(TIT);
grid on
end

