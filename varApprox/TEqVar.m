function [ vecOut ] = TEqVar( a, Va, b, Vb, R, VR )
%TEQVAR Summary of this function goes here
%   Detailed explanation goes here

% Ra = VarClass(a(5:7),Va(5:7));
% Ra = [1-Ra(2)^2/2 - Ra(3)^2/2, -Ra(3), Ra(2); Ra(3), 1-Ra(1)^2/2 - Ra(3)^2/2, -Ra(1); -Ra(2), Ra(1), 1-Ra(2)^2/2 - Ra(2)^2/2];
% Ra = VarClass(V2R(a(5:7)),reshape([Ra.var],3,3));

Rb = VarClass(b(5:7),Vb(5:7));
Rb = [-Rb(2)^2/2 - Rb(3)^2/2, -Rb(3), Rb(2); Rb(3), -Rb(1)^2/2 - Rb(3)^2/2, -Rb(1); -Rb(2), Rb(1), -Rb(1)^2/2 - Rb(2)^2/2];
Rb = VarClass(V2R(b(5:7)),reshape([Rb.var],3,3));

ta = VarClass(a(1:4),Va(1:4)); ta = ta(1:3).*repmat(ta(4),1,3);
%tb = VarClass(b(1:4),Vb(1:4)); tb = tb(1:3).*repmat(tb(4),1,3);

R = VarClass(R,VR);

eq1 = Rb-eye(3);
eq2 = R*ta';
vecOut = eq1\eq2;

end

