function [ o, Vo ] = MVar( a, Va, b, Vb )
%MVar estimates variance when two gaussian variables a and b are multiplied
%   together a*b
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   a- scalar, the 1st variable
%   Va- variance in a
%   b- scalar, the 2nd variable
%   Vb- variance in b
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   o- a*b
%   Vo- estimated variance of a*b
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

validateattributes(a,{'numeric'},{'scalar'});
validateattributes(Va,{'numeric'},{'scalar','positive'});
validateattributes(b,{'numeric'},{'scalar'});
validateattributes(Vb,{'numeric'},{'scalar','positive'});

a = double(a);
Va = double(Va);
b = double(b);
Vb = double(Vb);

o = a.*b;
Vo = Va.*Vb + (a.^2).*Vb + (b.^2).*Va;

end

