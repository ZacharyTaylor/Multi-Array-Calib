function [ vecOut ] = TEq2( a )
%TEQVAR Summary of this function goes here
%   Detailed explanation goes here

Ra = V2R(a(5:7));
ta = a(1:3)'*a(4);

eq1 = Ra-eye(3);
eq2 = ta;
vecOut = eq1\eq2;

end

