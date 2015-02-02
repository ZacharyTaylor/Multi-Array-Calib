function [ camData ] = FordCamInfo( fordPath, idx )
%FORDCAMINFO Sets the directory layout, masks and intrinsics for the ford
%   dataset
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   fordPath- path to where the ford dataset is stored
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
validateattributes(fordPath,{'char'},{'vector'});
validateattributes(idx,{'numeric'},{'scalar','integer','positive','nonzero'});
if(~exist(fordPath,7))
    error('%s is not a valid directory');
end

%set destination folder
switch idx
    case 1
        camData.folder = [fordPath '/Processed/ladybug/camera_0/'];
    case 2
        camData.folder = [fordPath '/Processed/ladybug/camera_1/'];
    case 3
        camData.folder = [fordPath '/Processed/ladybug/camera_2/'];
    case 4
        camData.folder = [fordPath '/Processed/ladybug/camera_3/'];
    case 5
        camData.folder = [fordPath '/Processed/ladybug/camera_4/'];
    otherwise
        error('%i is not a valid camera index',idx);
end

%intrinsic camera parameters
switch idx
    case 1
        camData.K = [408.397136 0 631.070016 0; 0 408.397136 806.586960 0; 0 0 1 0];
        camData.D = [0 0 0 0 0]';
    case 2
        camData.K = [402.206240 0 624.348224 0; 0 402.206240 784.646528 0; 0 0 1 0];
        camData.D = [0 0 0 0 0]';
    case 3
        camData.K = [398.799712 0 629.331664 0; 0 398.799712 818.201152 0; 0 0 1 0];
        camData.D = [0 0 0 0 0]';
    case 4
        camData.K = [406.131504 0 622.543536 0; 0 406.131504 820.718880 0; 0 0 1 0];
        camData.D = [0 0 0 0 0]';
    case 5
        camData.K = [400.730832 0 618.114496 0; 0 400.730832 796.724512 0; 0 0 1 0];
        camData.D = [0 0 0 0 0]';
    otherwise
        error('%i is not a valid camera index',idx);
end

%mask
switch idx
    case 1
        camData.mask = rgb2gray(imread([fordPath '/Processed/ladybug/masks/cam0.png'])) > 0;
    case 2
        camData.mask = rgb2gray(imread([fordPath '/Processed/ladybug/masks/cam1.png'])) > 0;
    case 3
        camData.mask = rgb2gray(imread([fordPath '/Processed/ladybug/masks/cam2.png'])) > 0;
    case 4
        camData.mask = rgb2gray(imread([fordPath '/Processed/ladybug/masks/cam3.png'])) > 0;
    case 5
        camData.mask = rgb2gray(imread([fordPath '/Processed/ladybug/masks/cam4.png'])) > 0;
    otherwise
        error('%i is not a valid camera index',idx);
end

%timestamps
camData.time = readTimeData([fordPath '/Processed/ladybug/timestamps.bin']);
   
end
        
        
        
        
        