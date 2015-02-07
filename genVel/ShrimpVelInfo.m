function [ velData ] = ShrimpVelInfo( shrimpPath )
%SHRIMPVELINFO Sets the directory layout and timestamps for the shrimp
%   dataset
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   shrimpPath- path to where the shrimp dataset is stored
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
validateattributes(shrimpPath,{'char'},{'vector'});
if(~exist(shrimpPath,'dir'))
    error('%s is not a valid directory');
end

%set destination folder
velData.folder = [shrimpPath '/Processed/velodyne/'];

%get files
velData.files = dir([velData.folder,'*.ply']);

%timestamps
velData.time = ReadTimeData([shrimpPath '/Processed/velodyne/timestamps.bin']);
   
end
        
        
        
        
        