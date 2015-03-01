function [ subVel, idx ] = SubVel( vel, sub )
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
if(size(vel,1) < sub)
    subVel = vel;
    idx = 1:size(vel,1);
    return;
end

%get distance
dist = sqrt(sum(vel(:,1:3).^2,2));

%get difference in readings
diff = max(abs(dist(2:end-1) - dist(3:end)),abs(dist(2:end-1) - dist(1:end-2)));
diff = [diff(1);diff;diff(end)];

%sort by distance
[~,diff] = sort(diff,'descend');

%get subsampled points
subA = floor(sub/2);
subB = sub-subA;

%get index of subsampled points
idx = [diff(1:subA); datasample(diff(subA+1:end),subB,1,'Replace',false)];
%idx = diff(1:sub);

%subsample
subVel = vel(idx,:);

end

