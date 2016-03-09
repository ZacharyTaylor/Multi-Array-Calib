function [] = CalibPath( set )
%CalibPath Adds or removes the Multi-Array-Calib folders from matlabs path
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   set- if true sets the path, otherwise removes it
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

if(nargin == 0)
    set = true;
end

validateattributes(set,{'logical'},{'scalar'});
pathFolders = {};

%contains functions for handling camera data and generating camera tforms
pathFolders = [pathFolders,'./genCam'];

%contains functions for handling visual inertial data and generating vi tforms
pathFolders = [pathFolders,'./genVIO'];

%contains functions for handling vicon data and generating vicon tforms
pathFolders = [pathFolders,'./genVicon'];

%contains functions for performing hand-eye calibartion
pathFolders = [pathFolders,'./handEye'];

%contains transformation representations and converters
pathFolders = [pathFolders,'./tforms'];

%contains timing offset functions and interpolation methods
pathFolders = [pathFolders,'./timing'];

%contains rough variance approximations used with translation estimates
pathFolders = [pathFolders,'./varApprox'];

%contains custom ros messages
pathFolders = [pathFolders,'./rosMsgs/matlab_gen/msggen'];

%holds the rest
pathFolders = [pathFolders,'./misc'];

for i = 1:length(pathFolders)
    if(set)
        addpath(pathFolders{i});
    else
        rmpath(pathFolders{i});
    end
end

%modifies matlabs java class-path to allow custom ros messages
javaaddpath('./rosMsgs/matlab_gen/jar/ros_vrpn_client-0.0.0.jar');

end

