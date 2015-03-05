function [ o, Vo ] = PVar( a, Va, p )
%PVar estimates variance when a gaussian variables a is taken to power p
%   a^p
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   a- scalar, the 1st variable
%   Va- variance in a
%   p- power to use
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   o- a^p
%   Vo- estimated variance of a^p
%--------------------------------------------------------------------------
%   References:
%--------------------------------------------------------------------------
%   This function is part of the Multi-Array-Calib toolbox 
%   https://github.com/ZacharyTaylor/Multi-Array-Calib
%   
%   This code was written by Zachary Taylor
%   zacharyjeremytaylor@gmail.com
%   http://www.zjtaylor.com

validateattributes(a,{'numeric'},{'scalar'});
validateattributes(Va,{'numeric'},{'scalar','positive'});
validateattributes(p,{'numeric'},{'scalar'});

a = double(a);
Va = double(Va);
p = double(p);

Vo = Va;
o = a;
for i = 2:p
    [o,Vo] = MVar(o,Vo,a,Va);
end

