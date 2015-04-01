function [ out, outV ] = CombEst( varargin )
%COMBEST combines a series of estimates + variance into a single esitmate
%--------------------------------------------------------------------------
%   Inputs:
%--------------------------------------------------------------------------
%   Requires an even number of inputs of the following type
%   val- nxm array of estimated values
%   var- nxm corrosponding variance
%
%   Example calls
%       CombEst(valA,varA)
%       CombEst(valA,varA,valB,varB)
%       CombEst(valA,varA,valB,varB,valC,varC,...)
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   val- nxm array of combined values
%   var- nxm corrosponding variance
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
for i = 1:ceil(length(varargin)/2)
    validateattributes(varargin{2*i-1},{'double'},{'size',size(varargin{1})});
    validateattributes(varargin{2*i},{'double'},{'size',size(varargin{1})});
end

base = varargin(1:2:end);
baseV = varargin(2:2:end);

out = zeros(size(base{1}));
outV = out;

%combine
for i = 1:length(base)
    baseV{i} = 1./baseV{i};
    out = out + base{i}.*baseV{i};
    outV = outV + baseV{i};
end

out = out./outV;
outV = 1./outV;

end

