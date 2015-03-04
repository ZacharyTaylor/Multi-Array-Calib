function [ vecOut, varOut ] = TEqVar2( a, Va )
%TEQVAR Summary of this function goes here
%   Detailed explanation goes here

eq1 = (V2R(a(5:7))-eye(3));
eq2 = (a(4)*a(1:3))';
vecOut = eq1\eq2;

(CrossMat(Va(5:7)).^2)

%invEq1Var =  Vb(5:7)./(b(5:7).^4);
%eq2Var = (a(4).^2)*Va(1:3) + (a(1:3).^2)*Va(4);
%eq2Var = (R.^2)*eq2Var' + VR*((a(4)*a(1:3))'.^2);

varOut = [1;1;1];%((1./b(5:7)').^2).*eq2Var + (eq2.^2).*invEq1Var';

end

