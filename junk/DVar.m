function [ V ] = DVar( a, Va, b, Vb )
%DVar estimates variance when two gaussian variables a and b are divided
%   together a/b
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
%   V- estimated variance of a/b
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

V = MVar(a,Va,1./b,IVar(b,Vb));

end

