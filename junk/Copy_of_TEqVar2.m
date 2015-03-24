function [ vecOut ] = TEqVar2( a, Va )
%TEQVAR Summary of this function goes here
%   Detailed explanation goes here

Ra = VarClass(a(5:7),Va(5:7));
Ra = [1-Ra(2)^2/2 - Ra(3)^2/2, -Ra(3), Ra(2); Ra(3), 1-Ra(1)^2/2 - Ra(3)^2/2, -Ra(1); -Ra(2), Ra(1), 1-Ra(2)^2/2 - Ra(2)^2/2];
Ra = VarClass(V2R(a(5:7)),reshape([Ra.var],3,3));

ta = VarClass(a(1:4),Va(1:4)); ta = ta(1:3).*repmat(ta(4),1,3);

eq1 = Ra-eye(3);
eq2 = ta';
vecOut = eq1\eq2;

end

