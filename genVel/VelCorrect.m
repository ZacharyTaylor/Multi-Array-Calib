function [ points ] = VelCorrect( points, tFrac, tform )
%VELCORRECT correct for motion during the recording of lidar points.
%   Assumes a static enviroment
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   points- nxm set of velodyne points where m >= 3
%   tFrac- nx1 vector of amount of time that has passed since the point was
%       recorded. Assumes that last frame occoured at -1 and next frame
%       will happen at 1
%   tform- 1x7 vector, tform from previous frame to this one
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   points- nxm set of velodyne points ajusted for timing offset
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
validateattributes(points,{'numeric'},{'2d'});
if(size(points,2) < 3)
    error('points must have atleast 3 columns');
end
validateattributes(tFrac,{'numeric'},{'numel',size(points,1),'>=',-1,'<=',1});
validateattributes(tform,{'numeric'},{'size',[1,7]});

%ensure inputs are of type doubles
points = double(points);
tFrac = double(tFrac(:));
tform = double(tform);

%get difference between poisitions
pointsDiff = V2T(tform)*[points(:,1:3), ones(size(points,1),1)]';
pointsDiff = pointsDiff(1:3,:)';
pointsDiff = pointsDiff - points(:,1:3);

%account for timing offset
points(:,1:3) = points(:,1:3) - repmat(tFrac,1,3).*pointsDiff;

end

