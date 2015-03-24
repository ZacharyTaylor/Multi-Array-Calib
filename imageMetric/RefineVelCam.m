function [ TVecOut, vTVecOut ] = RefineVelCam( TVecIn,vTVecIn,lidarInfo,camInfo,metric,numScans )
%REFINEVELCAM Summary of this function goes here
%   Detailed explanation goes here

%get scans to use
dataIdx = datasample(2:length(camInfo.files),numScans,'Replace',false);

%disable warning that gets spammed
warning('off','images:initSize:adjustingMag');

%% load images and scans
images = cell(numScans,2);
scans = cell(numScans,1);
for j = 1:numScans
    %previous image
    images{j,1} = ProcessImage(camInfo, dataIdx(j)-1, 2);
    %current image
    images{j,2} = ProcessImage(camInfo, dataIdx(j), 2);
    
    %matching lidar scan
    scans{j} = ProcessScan(lidarInfo, camInfo, dataIdx(j));
end

%load camera
K = camInfo.K;
K = gpuArray(single(K));

%setup inital mean and sd
sd = sqrt(vTVecIn)';
m = TVecIn';

%set up optimizer
opts.LBounds = [-3,-3,-3,-pi/2,-pi/2,-pi]'; opts.UBounds = [3,3,3,pi/2,pi/2,pi]'; 
opts.TolX = 1e-9;
opts.TolFun = 0.0001;
opts.SaveVariables = 'off';
opts.MaxIter = 500;
opts.DispFinal = 'off';
opts.DispModulo = inf;

%keep points in bounds
m(m > opts.UBounds) = opts.UBounds(m > opts.UBounds);
m(m < opts.LBounds) = opts.LBounds(m < opts.LBounds);

%keep sd reasonable
sd(sd > 0.7*opts.UBounds) = 0.7*opts.UBounds(sd > 0.7*opts.UBounds);

%run metric
TVecOut = cmaes(@(tform) RunColourMetric(tform, K, scans, images, 2), m,sd, opts);
TVecOut = fminsearch(@(tform) RunColourMetric(tform, K, scans, images,2), TVecOut);
TVecOut = TVecOut';

%find variance
vTVecOut = zeros(6,6);
diff = 0.1;
for i = 1:6
    scansO = cell(numScans,1);
    offset = zeros(1,6);
    offset(i) = diff;
    for j = 1:numScans
        %offset lidar scans
        scansO{j} = OffsetScan(scans{j}, lidarInfo, camInfo, dataIdx(j),offset);
    end

    vTVecOut(i,:) = fminsearch(@(tform) RunColourMetric(tform, K, scansO, images,2), TVecOut');
end

%convert scores to variance
vTVecOut = vTVecOut-repmat(TVecOut,6,1);
vTVecOut = (sum(vTVecOut.*vTVecOut/(diff*diff),1));

end

function [ image ] = ProcessImage(camInfo, idx, blur)

    %load image
    image = imread([camInfo.folder camInfo.files(idx).name]);
    if(size(image,3) == 3)
        image = rgb2gray(image);
    end
    
    %blur image
    G = fspecial('gaussian',[50 50],blur);
    image = imfilter(double(image),G,'same');
    
    %mask image
    if(isfield(camInfo,'mask'))
        image(~camInfo.mask) = 0;
    end
    
    %undistort image
    image = Undistort(double(image), camInfo.D, camInfo.K);
    
    %move to gpu
    image = gpuArray(uint8(image));
end

function [ scan ] = ProcessScan(lidarInfo, camInfo, idx)

    %match lidar with previous camera frame
    time = double(lidarInfo.time) - double(camInfo.time(idx-1));
    [~,idxPrev] = min(abs(time));
    TPrev = double(lidarInfo.time(idxPrev));
    
    %read lidar data
    [xyz, intensity, timeFrac] = ReadVelData([lidarInfo.folder lidarInfo.files(idxPrev).name]);
    
    %find time difference between scans
    time = double(lidarInfo.time) - double(camInfo.time(idx));
    [~,idxCurr] = min(abs(time));
    tCurr = double(lidarInfo.time(idxCurr));
    
    %correct for motion during scan
    points = VelCorrect(xyz, timeFrac, lidarInfo.T_Skm1_Sk(idxPrev,:));
    %points = xyz;
    
    %find transformation between scans
    tform = V2T(lidarInfo.T_S1_Sk(idxPrev,:))\V2T(lidarInfo.T_S1_Sk(idxCurr,:));
    
    %invert to get movement of coordinate frames
    tform = inv(tform);
    
    %get difference in points
    dPoints = (tform*[points, ones(size(points,1),1)]')';
    dPoints = dPoints(:,1:3) - points;
    
    %interpolate point positions
    tStartFrac = (double(camInfo.time(idx-1)-TPrev))/(tCurr - TPrev);
    tEndFrac = (double(camInfo.time(idx)-TPrev))/(tCurr - TPrev);
    %tStartFrac = 0;
    %tEndFrac = 1;
    
    %combine
    points = [points + tStartFrac*dPoints, points + tEndFrac*dPoints];
    
    %convert intensity to colours
    cmap = colormap(jet(256));
    intensity = cmap(round(255*intensity)+1,:);

    %move everything to the gpu
    scan = gpuArray(single([points, intensity]));
end

function [ scanOut ] = OffsetScan(scanIn, lidarInfo, camInfo, idx,offset)

    %match lidar with previous camera frame
    time = double(lidarInfo.time) - double(camInfo.time(idx-1));
    [~,idxPrev] = min(abs(time));
    TPrev = double(lidarInfo.time(idxPrev));
    
    %find time difference between scans
    time = double(lidarInfo.time) - double(camInfo.time(idx));
    [~,idxCurr] = min(abs(time));
    tCurr = double(lidarInfo.time(idxCurr));
    
    findT = @(A,B) (T2V(inv(V2T(A)\V2T(B))));
    
    %find transformation between scans and the variance to go with it
    [tform, tformV] = IndVar(0.01,findT,lidarInfo.T_S1_Sk(idxPrev,:),zeros(1,6),lidarInfo.T_S1_Sk(idxCurr,:),(lidarInfo.T_Var_S1_Sk(idxCurr,:)-lidarInfo.T_Var_S1_Sk(idxPrev,:)));
    
    %get final tform with offset
    tform = tform + offset.*sqrt(tformV);
    tform = V2T(tform);
    points = double(gather(scanIn(:,1:3)));
    
    %get difference in points
    dPoints = (tform*[points, ones(size(points,1),1)]')';
    dPoints = dPoints(:,1:3) - points;
    
    %interpolate point positions
    tStartFrac = (double(camInfo.time(idx-1)-TPrev))/(tCurr - TPrev);
    tEndFrac = (double(camInfo.time(idx)-TPrev))/(tCurr - TPrev);
    %tStartFrac = 0;
    %tEndFrac = 1;
    
    %combine
    points = [points + tStartFrac*dPoints, points + tEndFrac*dPoints];
    
    %move everything to the gpu
    scanOut = gpuArray(single([points, scanIn(:,7:end)]));
end

