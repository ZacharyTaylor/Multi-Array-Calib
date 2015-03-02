function [ T ] = V2T( V )
%V2T converts a 1x6 transformation vector into a 4x4 transformation matrix
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%    V - 1x6 vector of form [nx,ny,s,rx,ry,rz] where nx,ny,s is the 
%       translation. This is formed by converting the X,Y,Z position to
%       a normalized vector + scale [nx,ny,nz,s]. This is multiplied by the
%       sign of nz to give the final values. This system is now overdefined
%       as the value of nz can be uniquely recovered given [nx,ny,s] and so
%       is discarded.
%
%       rx,ry,rz is an angle-axis representation of the angle where the
%       unit vector representing the axis has been multipled by the angle 
%       of rotation about it
%
%       while a little unconventional this vector was found to be a clean
%       and compact way of representing transformation data that allowed
%       easy use in hand-eye calibration and allowed clear representation
%       of the unknown absolute scale of camera sensors
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%	T - a standard 4x4 transformation matrix
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

T = eye(4);
T(1:3,4) = V(3)*[V(1:2)';sqrt(1-V(1).^2-V(2).^2)];
T(1:3,1:3) = V2R(V(4:6));


end

