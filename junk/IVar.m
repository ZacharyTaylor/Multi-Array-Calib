function [ o, Vo ] = IVar( i, Vi )
%IVar estimates variance when a gaussian variable i is inverted 1/i
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   i- scalar, the variable
%   Vi- variance in i
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   o- 1/i
%   Vo- estimated variance of 1/i
%
%--------------------------------------------------------------------------
%   References:
%--------------------------------------------------------------------------
%   This function is part of the Multi-Array-Calib toolbox 
%   https://github.com/ZacharyTaylor/Multi-Array-Calib
%   
%   This code makes use of equations from
%   Stochastic Collection and Replenishment (SCAR): Objective Functions
%   By Andrew W Palmer, Andrew J Hill and Steven J Scheding
%
%   This code was written by Zachary Taylor
%   zacharyjeremytaylor@gmail.com
%   http://www.zjtaylor.com

validateattributes(i,{'numeric'},{'scalar','nonzero'});
validateattributes(Vi,{'numeric'},{'scalar','positive'});

i = double(i);
Vi = double(Vi);

o = 1./i;
Vo = Vi./(i.^4);

end

