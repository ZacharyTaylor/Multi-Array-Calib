function [ TOut, TBoot ] = OptValsSIFT(TMean,TVar, lidar, cam, invert)

%% get data
matchNum = 2;
dataNum = datasample(2:(size(lidar.files,1)-2),matchNum,'Replace',false);
%scale = 0.5;

%load images
images = cell(matchNum,1);


%previous image
for j = 1:matchNum
    A = imread([cam.folder cam.files(dataNum(j)-1).name]);
    B = imread([cam.folder cam.files(dataNum(j)).name]);
    if(size(A,3) == 3)
        A = rgb2gray(A);
        B = rgb2gray(B);
    end
    
    A = uint8(undistort(double(A), cam.D, cam.K(1:3,1:3)));
    B = uint8(undistort(double(B), cam.D, cam.K(1:3,1:3)));

    pA = detectMinEigenFeatures(A);
    pA = pA.Location;
    
    pointTracker = vision.PointTracker('MaxBidirectionalError', 2);
    initialize(pointTracker, pA, A);
    [pB, matches] = step(pointTracker, B);
    release(pointTracker);
    
    pA = pA(matches,:);
    pB = pB(matches,:);
    
    [~, inliers] = estimateFundamentalMatrix(pA,pB,'NumTrials',500);
    pA = pA(inliers,:);
    pB = pB(inliers,:);
    
    images{j} = [pA,pB];
end

images{end+1} = imread([cam.folder cam.files(dataNum(1)-1).name]);

%load camera
K = cam.K;
K = gpuArray(single(K));

%load scans
scans = cell(matchNum,1);
cmap = colormap(jet(256));
for i = 1:matchNum
    temp = dlmread([lidar.folder lidar.files(dataNum(i)-1).name],' ');
    
    tform = vec2tran(lidar.T_Skm1_Sk(dataNum(i)-1,:)');
    
    temp(:,1:3) = tformInterp3(tform,getTime(temp, 0),temp(:,1:3));
        
    comp = (tform*[temp(:,1:3), ones(size(temp,1),1)]')';
    
    temp = [temp(:,1:3), comp(:,1:3) temp(:,4)];
    
    scans{i} = temp;
    scans{i}(:,7:9) = cmap(round(255*scans{i}(:,7))+1,:);
    scans{i} = gpuArray(single(scans{i}));
end

rangeT = sqrt(TVar);

opts.LBounds = [-3,-3,-3,-pi/2,-pi/2,-pi]'; opts.UBounds = [3,3,3,pi/2,pi/2,pi]'; 
opts.TolX = 1e-9;
opts.TolFun = 0.0001;
opts.SaveVariables = 'off';
opts.MaxIter = 500;
opts.DispFinal = 'off';
opts.DispModulo = inf;

TMean(TMean > opts.UBounds) = opts.UBounds(TMean > opts.UBounds);
TMean(TMean < opts.LBounds) = opts.LBounds(TMean < opts.LBounds);
rangeT(rangeT > 0.7*opts.UBounds) = 0.7*opts.UBounds(rangeT > 0.7*opts.UBounds);

TOut = cmaes(@(tform) runFlowMetric2( tform, K, scans, images, invert ), TMean, rangeT, opts);

TOut = TOut(:);

TBoot = zeros(10,6);

% bootMax = 0;
% for i = 1:size(scans,1)
%     bootMax = max(bootMax,size(scans{i},1));
% end
% 
% for i = 1:10
%     i
%     boot = datasample(1:bootMax,bootMax);
%     boot = sort(boot);
%     [b,idx] = unique(boot);
%     idx = [idx;bootMax+1];
%     boot = zeros(bootMax,1);
%     boot(b) = diff(idx);
% 
%     opts.MaxIter = 100;
%     TBoot(i,:) = cmaes(@(tform) runFlowMetricBoot( tform, K, scans, images, boot, invert), TMean, rangeT, opts);
% end
% TBoot = var(TBoot)';

% for i = 1:matchNum
%     opts.MaxIter = 100;
%     TBoot(i,:) = cmaes(@(tform) runFlowMetric( tform, K, scans(i,:), images(i,:), invert ), TOut, rangeT, opts);
%     %TBoot(i,:) = fminsearch(@(tform) runFlowMetric( tform, K, scans(i,:), images(i,:), invert), TOut);
% end
% TBoot = var(TBoot)';


