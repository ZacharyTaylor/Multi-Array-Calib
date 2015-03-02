function [ V ] = R2V( R )
%R2V converts a 3x3 rotation matrix into a 1x3 angle-axis vector 
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%	R - a standard 3x3 transformation matrix
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   V - 1x3 vector of form [rx,ry,rz] where rx,ry,rz is an angle-axis 
%       representation of the angle where the unit vector representing the
%       axis has been multipled by the angle of rotation about it
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

validateattributes(R, {'numeric'},{'size',[3,3]});

R = vrrotmat2vec(double(R));
V = R(1:3)*R(4);
V = V';
end


