function [ V ] = IRMIVar( RVec, RVar )
%IRMIVar estimates the variance of inv(R-I)
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   RVec- 1x3 rotation vector (must be near zero as we make use of the
%       small angle approximation)
%   RVar- 1x3 vector giving variance of each element of RVec
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   V- estimated variance of inv(R-I)
%
%--------------------------------------------------------------------------
%   References:
%--------------------------------------------------------------------------
%   This function is part of the Multi-Array-Calib toolbox 
%   https://github.com/ZacharyTaylor/Multi-Array-Calib
%   
%   This code was written by Zachary Taylor
%   zacharyjeremytaylor@gmail.com
%   http://www.zjtaylor.com

%form variance objects
A = VarClass(RVec(1),RVar(1));
B = VarClass(RVec(2),RVar(2));
C = VarClass(RVec(3),RVar(3));

%% find det(R-I)

%if RVec = [a,b,c] det(R-I) is roughly - (a^4*b^2)/8 - (a^4*c^2)/8 - (a^2*b^4)/8 - (a^2*b^2*c^2)/4 - a^2*b^2 - (a^2*c^4)/8 - a^2*c^2 - (b^4*c^2)/8 - (b^2*c^4)/8 - b^2*c^2

d = - (A^4*B^2)/8 - (A^4*C^2)/8 - (A^2*B^4)/8 - (A^2*B^2*C^2)/4 - A^2*B^2 - (A^2*C^4)/8 - A^2*C^2 - (B^4*C^2)/8 - (B^2*C^4)/8 - B^2*C^2;

%% finding inv(R-I)
v = [ A^4/4 + (A^2*B^2)/4 + (A^2*C^2)/4 + A^2 + (B^2*C^2)/4,                           A*B - (B^2*C)/2 - (A^2*C)/2,                           (B*A^2)/2 + A*C + (B*C^2)/2;...
                                (C*A^2)/2 + A*B + (C*B^2)/2, (A^2*B^2)/4 + (A^2*C^2)/4 + B^4/4 + (B^2*C^2)/4 + B^2,                           B*C - (A*C^2)/2 - (A*B^2)/2;...
                                A*C - (B*C^2)/2 - (A^2*B)/2,                           (A*B^2)/2 + B*C + (A*C^2)/2, (A^2*B^2)/4 + (A^2*C^2)/4 + (B^2*C^2)/4 + C^4/4 + C^2];

v = v/d;                            
end

