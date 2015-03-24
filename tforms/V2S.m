function [ S ] = V2S( V )
%V2S converts a 1x6 transformation vector into a 1x7 transformation vector
%   with normalized distances + scale (usful for handling camera estimates)
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%    V - 1x6 vector of form [x,y,z,rx,ry,rz] where [x,y,z] is the 
%       translation. [rx,ry,rz] is an angle-axis representation of the 
%       angle where the unit vector representing the axis has been
%       multipled by the angle of rotation about it
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%	S - 1x7 vector of form [nx,ny,nz,s,rx,ry,rz] where [nx,ny,nz] is the 
%       normalized translation and s is the scale. [rx,ry,rz] is an 
%       angle-axis representation of the angle where the unit vector
%       representing the axis has been multipled by the angle of rotation
%       about it
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

validateattributes(V, {'numeric'},{'size',[1,6]});

V = double(V);

S = zeros(1,7);
S(5:7) = V(4:6);
S(4) = norm(V(1:3));
S(1:3) = V(1:3)/S(4);

S(~isfinite(S)) = 0;

end

