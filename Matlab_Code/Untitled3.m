ras = @(x, y) 20 + x.^2 + y.^2 - 10*(cos(2*pi*x) + cos(2*pi*y));

rf3 = @(x, y) ras(x/10, y/10);
fsurf(rf3,[-30 30],"ShowContours","on")
title("rastriginsfcn([x/10,y/10])")
xlabel("x")
ylabel("y")
rng default % For reproducibility
x = optimvar("x","LowerBound",-70,"UpperBound",130);
y = optimvar("y","LowerBound",-70,"UpperBound",130);
prob = optimproblem("Objective",rf3(x,y));
options = optimoptions("surrogateopt","PlotFcn",[]);
[solsur,fvalsur,eflagsur,outputsur] = solve(prob,...
    "Solver","surrogateopt",...
    "Options",options);