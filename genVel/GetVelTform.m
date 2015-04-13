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
%   tformPrev- 1x7 transformation vector from frame-2 to frame-1
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
%[velCurr,idx] = SubVel(velCurr,subSample);
%tFracCurr = tFracCurr(idx);
% [velPrev,idx] = SubVel(velPrev,subSample);
% tFracPrev = tFracPrev(idx);

%correct previous scans motion error
velPrev = VelCorrect(velPrev, tFracPrev, tformPrev);

%correct current scans motion error (will have a bit of error)
velCurr = VelCorrect(velCurr, tFracCurr, tformPrev);

%find transformation
%tform = icpMexTime(velPrev',velCurr',inv(V2T(tformPrev)),tFracCurr',0.3,'point_to_plane');
tform = icpMex(velPrev',velCurr',inv(V2T(tformPrev)),0.5,'point_to_plane');
tform = T2V(inv(tform));

if(norm(tform(1:3)) > 3)
    tform(1:6) = 0;
end

%find variance
step = 0.001;
velPT = (V2T(tform)*[velPrev, ones(size(velPrev,1),1)]')'; velPT = velPT(:,1:3);
idx = knnsearch(velPT,velCurr);

dxx = zeros(length(tform));
for i = 1:length(tform)
    for j = 1:length(tform)
        temp = tform; 
        temp(j) = temp(j) + step;
        temp(i) = temp(i) + step;
        velPT = (V2T(temp)*[velPrev, ones(size(velPrev,1),1)]')'; velPT = velPT(:,1:3);
        err = sum((velPT(idx,:) - velCurr).^2,2);
        f1 = sum(err);

        temp = tform; 
        temp(j) = temp(j) + step;
        temp(i) = temp(i) - step;
        velPT = (V2T(temp)*[velPrev, ones(size(velPrev,1),1)]')'; velPT = velPT(:,1:3);
        err = sum((velPT(idx,:) - velCurr).^2,2);
        f2 = sum(err);

        temp = tform; 
        temp(j) = temp(j) - step;
        temp(i) = temp(i) + step;
        velPT = (V2T(temp)*[velPrev, ones(size(velPrev,1),1)]')'; velPT = velPT(:,1:3);
        err = sum((velPT(idx,:) - velCurr).^2,2);
        f3 = sum(err);

        temp = tform; 
        temp(j) = temp(j) - step;
        temp(i) = temp(i) - step;
        velPT = (V2T(temp)*[velPrev, ones(size(velPrev,1),1)]')'; velPT = velPT(:,1:3);
        err = sum((velPT(idx,:) - velCurr).^2,2);
        f4 = sum(err);

        dxx(i,j) = (f1-f2-f3+f4)/(4*step*step);
    end
end

dx = zeros(length(tform),1);
for i = 1:length(tform)
    temp = tform; 
    temp(i) = temp(i) + step;
    velPT = (V2T(temp)*[velPrev, ones(size(velPrev,1),1)]')'; velPT = velPT(:,1:3);
    err = sum((velPT(idx,:) - velCurr).^2,2);
    f1 = sum(err);
        
    temp = tform; 
    temp(i) = temp(i) - step;
    velPT = (V2T(temp)*[velPrev, ones(size(velPrev,1),1)]')'; velPT = velPT(:,1:3);
    err = sum((velPT(idx,:) - velCurr).^2,2);
    f2 = sum(err);

    dx(i) = (f1-f2)/(2*step);
end

dz = zeros(size(velCurr));
for i = 1:size(velCurr,2)
    temp = velCurr; 
    temp(:,i) = temp(:,i) + step;
    velPT = (V2T(tform)*[velPrev, ones(size(velPrev,1),1)]')'; velPT = velPT(:,1:3);
    err = sum((velPT(idx,:) - temp).^2,2);
    f1 = sum(err);
    
    temp = velCurr; 
    temp(:,i) = temp(:,i) - step;
    velPT = (V2T(tform)*[velPrev, ones(size(velPrev,1),1)]')'; velPT = velPT(:,1:3);
    err = sum((velPT(idx,:) - temp).^2,2);
    f2 = sum(err);
    
    dz(:,i) = (f1-f2)/(2*step);
end
dz = dz(:);

dxz = zeros(size(dx(:),1),size(dz(:),1));
for i = 1:size(dx(:),1)
    for j = 1:size(dz(:),1)
        dxz(i,j) = dx(i) + dz(j);
    end
end

d = dxx\dxz;
d = 0.05*0.05*(d*d');

tformVar = diag(d)';


% %bootstrap scans
% bootnum = 100;
% numPoints = ceil(min(size(velCurr,1),size(velPrev,1))/bootSample);
% tformVar = zeros(bootnum,6);
% for i = 1:bootnum
%     [subVelCurr,idx] = datasample(velCurr,numPoints);
%     subTFracCurr = tFracCurr(idx);
%     subVelPrev = datasample(velPrev,numPoints);
% 
%     %find bootstrap tform
%     %tformVar(i,:) = T2V(inv(icpMexTime(subVelPrev',subVelCurr',inv(V2T(tform)),subTFracCurr',0.3,'point_to_point')))';
%     tformVar(i,:) = T2V(inv(icpMex(subVelPrev',subVelCurr',inv(V2T(tform)),0.5,'point_to_point')))';
% end
% 
% %find variance
% tformVar = var(tformVar);

end

