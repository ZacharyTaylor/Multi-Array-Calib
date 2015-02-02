function [ velData ] = FordVelInfo( fordPath )
%FORDVELINFO Sets the directory layout and timestamps for the ford dataset
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   fordPath- path to where the ford dataset is stored
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
validateattributes(fordPath,{'char'},{'vector'});
if(~exist(fordPath,'dir'))
    error('%s is not a valid directory');
end

%set destination folder
velData.folder = [fordPath '/Processed/velodyne/'];

%get files
velData.files = dir([velData.folder,'*.ply']);

%timestamps
velData.time = readTimeData([fordPath '/Processed/velodyne/timestamps.bin']);
   
end
        
        
        
        
        