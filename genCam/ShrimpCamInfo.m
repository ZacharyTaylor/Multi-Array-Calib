function [ camData ] = ShrimpCamInfo( shrimpPath, idx )
%FORDCAMINFO Sets the directory layout, masks and intrinsics for the shrimp
%   dataset
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   shrimpPath- path to where the shrimp dataset is stored
%   idx- index of the camera to use (1-5)
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   camData- struct holding camera information
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

camData = struct;

%check inputs
validateattributes(shrimpPath,{'char'},{'vector'});
validateattributes(idx,{'numeric'},{'scalar','integer','positive','nonzero'});
if(~exist(shrimpPath,'dir'))
    error('%s is not a valid directory');
end

%set destination folder
switch idx
    case 1
        camData.folder = [shrimpPath '/Processed/ladybug/camera_0/'];
    case 2
        camData.folder = [shrimpPath '/Processed/ladybug/camera_1/'];
    case 3
        camData.folder = [shrimpPath '/Processed/ladybug/camera_2/'];
    case 4
        camData.folder = [shrimpPath '/Processed/ladybug/camera_3/'];
    case 5
        camData.folder = [shrimpPath '/Processed/ladybug/camera_4/'];
    case 6
        camData.folder = [shrimpPath '/Processed/ladybug/camera_5/'];
    otherwise
        error('%i is not a valid camera index',idx);
end

%intrinsic camera parameters
switch idx
    case 1
        camData.K = [403.431 0 621 0; 0 403.431 811.810 0; 0 0 1 0];
        camData.D = [0 0 0 0 0]';
    case 2
        camData.K = [410.810 0 623 0; 0 410.810 810.002 0; 0 0 1 0];
        camData.D = [0 0 0 0 0]';
    case 3
        camData.K = [412.206 0 628 0; 0 412.206 810.771 0; 0 0 1 0];
        camData.D = [0 0 0 0 0]';
    case 4
        camData.K = [409.433 0 609 0; 0 409.433 816.045 0; 0 0 1 0];
        camData.D = [0 0 0 0 0]';
    case 5
        camData.K = [404.472 0 627 0; 0 404.472 826.407 0; 0 0 1 0];
        camData.D = [0 0 0 0 0]';
    case 6
        camData.K = [408.488 0 617 0; 0 408.488 802.263 0; 0 0 1 0];
        camData.D = [0 0 0 0 0]';
    otherwise
        error('%i is not a valid camera index',idx);
end

%mask
switch idx
    case 1
        camData.mask = rgb2gray(imread([shrimpPath '/Processed/ladybug/masks/cam0.png'])) > 0;
    case 2
        camData.mask = rgb2gray(imread([shrimpPath '/Processed/ladybug/masks/cam1.png'])) > 0;
    case 3
        camData.mask = rgb2gray(imread([shrimpPath '/Processed/ladybug/masks/cam2.png'])) > 0;
    case 4
        camData.mask = rgb2gray(imread([shrimpPath '/Processed/ladybug/masks/cam3.png'])) > 0;
    case 5
        camData.mask = rgb2gray(imread([shrimpPath '/Processed/ladybug/masks/cam4.png'])) > 0;
    case 6
        camData.mask = rgb2gray(imread([shrimpPath '/Processed/ladybug/masks/cam5.png'])) > 0;
    otherwise
        error('%i is not a valid camera index',idx);
end

%timestamps
camData.time = ReadTimeData([shrimpPath '/Processed/ladybug/timestamps.bin']);
   
end
        
        
        
        
        