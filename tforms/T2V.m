function [ V ] = T2V( T )
%T2V converts a 4x4 transformation matrix into a 1x6 transformation vector
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   T - a standard 4x4 transformation matrix
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%    V - 1x6 vector of form [x,y,z,rx,ry,rz] where [x,y,z] is the 
%       translation. [rx,ry,rz] is an angle-axis representation of the 
%       angle where the unit vector representing the axis has been
%       multipled by the angle of rotation about it
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
%
%   Some of the lines and ideas were originally borrowed from 
%   http://asrl.utias.utoronto.ca/code/barfoot_tro14.zip and the paper
%   "Associating Uncertainty with Three-Dimensional Poses for use in
%   Estimation Problems". Though I belive apart from both representations
%   using angle axis systems there is now little similarity

validateattributes(T, {'numeric'},{'size',[4,4]});

T = double(T);

V(1:3) = T(1:3,4)';
V(4:6) = R2V(T(1:3,1:3));

V(isnan(V)) = 0;

end

