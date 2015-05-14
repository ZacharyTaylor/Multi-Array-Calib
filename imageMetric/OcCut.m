function [ vel ] = OcCut( vel )
%SUBVEL subsamples velodyne taking most salient points as determined by a
%   difference in distance metric + some random points
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   vel- nxm set of velodyne points where m >= 3
%   sub- number of points to subsample down to
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   subVel- subxm set of velodyne points
%   idx- subx1 index of the velodyne points
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
validateattributes(vel,{'numeric'},{'2d'});
if(size(vel,2) < 3)
    error('vel must have atleast 3 columns');
end

%get distance
dist = sqrt(sum(vel(:,1:3).^2,2));

%project onto sphere
sphere = zeros(size(vel,1),2);
sphere(:,1) = atan2(vel(:,2), vel(:,1));
sphere(:,2) = atan(vel(:,3)./ sqrt(vel(:,2).^2 + vel(:,1).^2));

%find closest points
idx = knnsearch(sphere,sphere,'k',50);
idx = idx(:,2:end);
diff = reshape(dist(idx),size(idx)) - repmat(dist,1,size(idx,2));
diff = min(diff,[],2)./dist;

vel = vel(diff > -0.1,:);

end

