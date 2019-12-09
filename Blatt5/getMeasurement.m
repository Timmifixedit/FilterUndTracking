function z = getMeasurement(x)
%measurement z = (x,y,gamma)
%state: X = (x,y,v,phi,w,a)

var = [0.1 0.1 0.01]';

M = [1 0 0 0;
     0 1 0 0;
     0 0 0 1];

H = [M zeros(size(M,1),size(x,1)-size(M,2));];

z = H*x + randn(size(M,1),1).*sqrt(var);
