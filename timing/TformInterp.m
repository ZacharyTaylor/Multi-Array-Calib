function [ Tout ] = TformInterp( T, interVal )
%TFORMINTERP Interpolate between eye(4) and a tform matrix T using slerp
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   T- 4x4 tform matrix
%   interVal- ratio to interpolate to (0 to 1), can also extrapolate
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   Tout- 4x4 output tform matrix
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

%check inputs
validateattributes(T,{'numeric'},{'size',[4,4]});
validateattributes(interVal,{'numeric'},{'scalar'});

%setup output
Tout = eye(4);

%interpolate translation
Tout(1:3,4) = interVal*T(1:3,4);

%interpolate rotation
quat1 = [1,0,0,0];
quat2 = dcm2quat(T(1:3,1:3));

theta = acos(dot(quat1,quat2));

if(theta == 0)
    return;
end

A = sin((1-interVal)*theta)/sin(theta);
B = sin(interVal*theta)/sin(theta);

out = A*quat1 + B*quat2;

Tout(1:3,1:3) = quat2dcm(out);

end

