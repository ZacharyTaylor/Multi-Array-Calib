function [ tform, tformVar ] = GetVelTform( velCurr, velPrev, tFracCurr, tFracPrev, tformPrev, subSample, bootSample)
%GENCAM Finds transformation between velodyne frames using ICP while
%   accounting for motion offsets
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   velCurr- nx3 the points of the current velodyne scan
%   velPrev- mx3 the points of the previous velodyne scan
%   tFracCurr- nx1 fractional times at which the point positions were
%       obtained. Set so that the velodyne head takes 1 to do a full
%       rotation thus if 0 is the centre of the current frame, -1 is the
%       centre of the previous frame. 
%   tFracPrev- mx1 fractional time for previous scan, same definition as
%       tFracCurr
%   tformPrev- 1x6 transformation vector from frame-2 to frame-1
%   subSample- scalar, number of points to subsample to before performing 
%       matching
%   bootSample- will only using 1 in bootSamlpe points of subsampled
%       velodyne scan for bootstrap matching
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   tfrom- 1x6 transformation vector from frame-1 to frame
%   tformVar- 1x6 vector giving the variance in tform
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
validateattributes(velCurr,{'numeric'},{'ncols',3});
validateattributes(velPrev,{'numeric'},{'ncols',3});
validateattributes(tFracCurr,{'numeric'},{'numel',size(velCurr,1),'>=',-1,'<=',1});
validateattributes(tFracPrev,{'numeric'},{'numel',size(velPrev,1),'>=',-1,'<=',1});
validateattributes(tformPrev,{'numeric'},{'size',[1,6]});
validateattributes(subSample,{'numeric'},{'scalar','positive','nonzero'});
validateattributes(bootSample,{'numeric'},{'scalar','positive','nonzero'});

%ensure inputs are of type doubles
velCurr = double(velCurr);
velPrev = double(velPrev);
tFracCurr = double(tFracCurr(:));
tFracPrev = double(tFracPrev(:));
tformPrev = double(tformPrev);

%subsample velodyne lidar down to subsample most salient points
[velCurr,idx] = SubVel(velCurr,subSample);
tFracCurr = tFracCurr(idx);
velPrev = SubVel(velPrev,subSample);
tFracPrev = tFracPrev(idx);

%correct previous scans motion error
velPrev = VelCorrect(velPrev, tFracPrev, tformPrev);

%find transformation
tform = icpMex(velPrev',velCurr',inv(V2T(tformPrev)),tFracCurr',0.5,'point_to_plane');
tform = T2V(inv(tform));

%bootstrap scans
bootnum = 100;
numPoints = subSample/bootSample;
tformVar = zeros(bootnum,6);
for i = 1:bootnum
    [subVelCurr,idx] = datasample(velCurr,numPoints);
    subTFracCurr = tFracCurr(idx);
    subVelPrev = datasample(velPrev,numPoints);

    %find bootstrap tform
    tformVar(i,:) = T2V(inv(icpMex(subVelPrev',subVelCurr',inv(V2T(tform)),subTFracCurr',0.5,'point_to_point')))';
end

%find variance
tformVar = var(tformVar);

end

