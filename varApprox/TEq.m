function [ vecOut ] = TEq(a, b, R)
%TEQVAR Summary of this function goes here
%   Detailed explanation goes here

Rb = V2R(b(5:7));
ta = a(1:3)'*a(4);

R = V2R(R);

eq1 = Rb-eye(3);
eq2 = R*ta;
vecOut = eq1\eq2;

end

