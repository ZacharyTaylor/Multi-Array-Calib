function [ timeFrac, valid ] = GenVelTimeFrac( xyz )
%GENVELTIMEFRAC Estimates velodyne time fraction for scans where the
%   timing information has been lost
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   xyz- nx3 list of 3d velodyne points, uses shrimp corrdinates (-x is 
%       foward, +y is right, -z is up
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   timeFrac- nx1 time fraction
%   valid- nx1 list of valid points (ignores ends of scan)
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
validateattributes(xyz,{'numeric'},{'ncols',3});

%ensure of type double
xyz = double(xyz);

%project onto sphere
sphere = zeros(size(xyz,1),2);
sphere(:,1) = atan2(xyz(:,2), -xyz(:,1));
sphere(:,2) = atan(xyz(:,3)./ sqrt(xyz(:,2).^2 + xyz(:,1).^2));

% %discretize
% sphere = 5000*sphere/(2*pi);
% sphere(:,1) = sphere(:,1) + 2500;
% sphere(:,2) = sphere(:,2) + 50;
% sphere = ceil(sphere);
% sphere = max(sphere,1);
% 
% idx = sphere(:,2) + (sphere(:,1)-1)*400;
% 
% %find timeFrac
% load('timingMatrix.mat');
% timeFrac = timeMatrix(idx);

timeFrac = 0.5*sphere(:,1)./pi;

%find valid
valid = true(size(timeFrac));
valid(timeFrac(:,1) < -0.45) = false;
valid(timeFrac(:,1) > 0.45) = false;
