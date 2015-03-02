function [ R ] = V2R( V )
%V2R converts a 1x3 angle-axis vector into a 3x3 rotation matrix
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   V - 1x3 vector of form [rx,ry,rz] where rx,ry,rz is an angle-axis 
%       representation of the angle where the unit vector representing the
%       axis has been multipled by the angle of rotation about it
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%	R - a standard 3x3 transformation matrix
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
%   Estimation Problems".

validateattributes(V, {'numeric'},{'size',[1,3]});

V = double(V(:));

s = norm(V);
if(s == 0)
    R = eye(3);
    return;
end

V = [V/s; s];
R = vrrotvec2mat(V);

end

