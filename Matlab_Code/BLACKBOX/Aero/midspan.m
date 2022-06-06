function [y]=midspan(x)
n=length(x);
y=zeros(1,n-1);
for j=1:n-1
    y(j)=0.5*(x(j)+x(j+1));
end
