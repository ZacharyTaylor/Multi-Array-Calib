function [ velData ] = KittiVelInfo( kittiPath )
%KITTIVELINFO Sets the directory layout and timestamps for the kitti
%   dataset
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   kittiPath- path to where the kitti dataset is stored
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   velData- struct holding velodyne information
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

velData = struct;

%check inputs
validateattributes(kittiPath,{'char'},{'vector'});
if(~exist(kittiPath,'dir'))
    error('%s is not a valid directory');
end

%set destination folder
velData.folder = [kittiPath '/velodyne_points/data/'];

%get files
velData.files = dir([velData.folder,'*.txt']);

%timestamps
[velData.time, ~, ~] = ReadKittiTimestamps([velData.folder '../']);

end
        
        
        
        
        