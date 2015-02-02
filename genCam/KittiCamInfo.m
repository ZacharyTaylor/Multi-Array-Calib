function [ camData ] = KittiCamInfo( kittiPath, idx )
%FORDCAMINFO Sets the directory layout, masks and intrinsics for the kitti
%   dataset
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   kittiPath- path to where the kitti dataset is stored
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
validateattributes(kittiPath,{'char'},{'vector'});
validateattributes(idx,{'numeric'},{'scalar','integer','positive','nonzero'});
if(~exist(kittiPath,'dir'))
    error('%s is not a valid directory');
end

%set destination folder
switch idx
    case 1
        camData.folder = [kittiPath 'image_00/data/'];
    case 2
        camData.folder = [kittiPath 'image_01/data/'];
    case 3
        camData.folder = [kittiPath 'image_02/data/'];
    case 4
        camData.folder = [kittiPath 'image_03/data/'];
    otherwise
        error('%i is not a valid camera index',idx);
end

%intrinsic camera parameters
switch idx
    case 1
        camData.K = [979.92,0,690,0;0,974.1183,248.6443,0;0,0,1,0];
        camData.D = [-3.745594e-01 2.049385e-01 1.110145e-03 1.379375e-03 -7.084798e-02]';
    case 2
        camData.K = [990.3522,0,702,0;0,985.5674,260.7319,0;0,0,1,0];
        camData.D = [-3.712084e-01 1.978723e-01 -3.709831e-05 -3.440494e-04 -6.724045e-02]';
    case 3
        camData.K = [960.1149,0,694.7923,0;0,954.8911,240.3547,0;0,0,1,0];
        camData.D = [-3.685917e-01 1.928022e-01 4.069233e-04 7.247536e-04 -6.276909e-02]';
    case 4
        camData.K = [904.9931,0,695.7698,0;0,900.4945,238.982,0;0,0,1,0];
        camData.D = [-3.735725e-01 2.066816e-01 -6.133284e-04 -1.193269e-04 -7.600861e-02]';
    otherwise
        error('%i is not a valid camera index',idx);
end

%mask
camData.mask = true(512,1392);

%timestamps
camData.time = ReadKittiTimestamps([camData.folder,'../']);
   
end
        
        
        
        
        