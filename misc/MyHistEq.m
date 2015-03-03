function [ out ] = MyHistEq( in )
%MYHISTEQ evenly distributes data between 0 and 1, note does not prevserve
%   equivalence
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   in- data to equalize
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   out- equalized data
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

validateattributes(in,{'numeric'},{});

in = double(in);
in = in - min(in(:));

nz = find(in);

in(nz) = in(nz) - min(in(nz));
in(nz) = in(nz) / max(in(nz));

[~,temp] = sort(in(nz));

out = (0:1/(length(in(nz))-1):1)';
in(nz(temp)) = out;

out = in;

end

