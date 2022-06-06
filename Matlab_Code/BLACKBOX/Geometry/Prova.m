clear all
close all

figure()
for i=1:2*num-1
    plot3(P_tot(3*i-2,:),P_tot(3*i-1,:),P_tot(3*i,:),'*r');
    hold on
    axis equal
end
% Start assembly each solid
figure()
n_prec=0;
tr=[];
xyz=[];
for i=2:2*num-1
    P_prec=P_tot(3*(i-1)-2:3*(i-1),:);
    P_succ=P_tot(3*i-2:3*i,:);
    P_i=[P_prec';P_succ'];
    shp = alphaShape(P_i,1);
    [tr_i, xyz_i] = boundaryFacets(shp);
    n_succ=length(xyz_i)+n_prec;
        tr=[tr;tr_i+n_prec];
    xyz=[xyz;xyz_i];
    n_prec=n_succ;
    plot(shp)
    hold on
end

pr_fin=triangulation(tr,xyz);
stlwrite(pr_fin, 'Boom.stl');