clear all
P=[0 0 0; 10 0 0; 10 10 0; 0 10 0; 0 0 10; 10 0 10; 10 10 10; 0 10 10;];
shp = alphaShape(P);
[tr_i, xyz_i] = boundaryFacets(shp);
plot(shp)
RBP=RigidBodyParams(triangulation(tr_i,xyz_i));