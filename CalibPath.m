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

validateattributes(set,{'logical'},{'scalar'});
pathFolders = {};

%contains functions for handling camera data and generating camera tforms
pathFolders = [pathFolders,'./genCam'];

%contains functions for handling velodyne data and generating vel tforms
pathFolders = [pathFolders,'./genVel'];

%contains functions for handling nav data and generating nav tforms
pathFolders = [pathFolders,'./genNav'];

%contains functions for performing hand-eye calibartion
pathFolders = [pathFolders,'./handEye'];

%contains functions for matching the output of overlaping sensors
pathFolders = [pathFolders,'./imageMetric'];

%contains transformation representations and converters
pathFolders = [pathFolders,'./tforms'];

%contains timing offset functions and interpolation methods
pathFolders = [pathFolders,'./timing'];

%holds precalculated transformations
pathFolders = [pathFolders,'./storedTforms'];

%holds results
pathFolders = [pathFolders,'./results'];

%holds tests run for paper
pathFolders = [pathFolders,'./paperTests'];

%holds plots for paper
pathFolders = [pathFolders,'./paperPlots'];

for i = 1:length(pathFolders)
    if(set)
        addpath(pathFolders{i});
    else
        rmpath(pathFolders{i});
    end
end

end

