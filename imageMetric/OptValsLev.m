function [ TOut, TBoot ] = OptValsLev(TMean,TVar,lidar,cam, invert,matchNum)

%% get data
dataNum = datasample(1:size(lidar.files,1),matchNum,'Replace',false);

%load images
images = cell(matchNum+1,1);
 
for j = 1:matchNum
    images{j} = imread([cam.folder cam.files(dataNum(j)).name]);
    if(size(images{j},3) == 3)
        images{j} = rgb2gray(images{j});
    end
    images{j} = undistort(double(images{j}), cam.D, cam.K(1:3,1:3));
    
    if(j == 1)
        images{end} = images{1};
    end
    
    images{j} = LevImage(single(images{j}));
    images{j} = gpuArray(single(images{j}));

    if(j == 1)
        avImage = images{j};
    else
        avImage = avImage + images{j};
    end
end

avImage = avImage./matchNum;
for j = 1:matchNum
    images{j} = images{j} - avImage;
end

%load camera
K = cam.K;
K = gpuArray(single(K));

%load scans
scans = cell(matchNum,1);
cmap = colormap(jet(256));
for i = 1:matchNum
    time = double(lidar.time) - double(cam.time(dataNum(i)));
    [~,idx] = min(abs(time));
    t = double(lidar.time(idx));
    
    xyz = ReadVelData([lidar.folder lidar.files(idx).name]);
    
    scans{i} = LevLidar(xyz);
    scans{i}(:,4) = scans{i}(:,4)./max(scans{i}(:,4));
    scans{i} = [scans{i}(:,1:4),cmap(round(255*scans{i}(:,4))+1,:)];
    
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
TMean = TMean(:);
rangeT = rangeT(:);

TMean(TMean > opts.UBounds) = opts.UBounds(TMean > opts.UBounds);
TMean(TMean < opts.LBounds) = opts.LBounds(TMean < opts.LBounds);
rangeT(rangeT > 0.7*opts.UBounds) = 0.7*opts.UBounds(rangeT > 0.7*opts.UBounds);

TOut = cmaes(@(tform) runLevMetric( tform, K, scans, images, invert), TMean, rangeT, opts);

TOut = TOut(:);

TBoot = zeros(matchNum,6);

for i = 1:matchNum
    opts.MaxIter = 100;
    TBoot(i,:) = cmaes(@(tform) runLevMetric( tform, K, scans(i,:), images(i,:), invert ), TOut, rangeT, opts);
end
TBoot = var(TBoot)';

%TBoot = zeros(matchNum,6);
% 
% bootMax = 0;
% for i = 1:size(scans,1)
%     bootMax = max(bootMax,size(scans{i},1));
% end

% for i = 1:matchNum
% %     i
% %     boot = datasample(1:bootMax,bootMax);
% %     boot = sort(boot);
% %     [b,idx] = unique(boot);
% %     idx = [idx;bootMax+1];
% %     boot = zeros(bootMax,1);
% %     boot(b) = diff(idx);
% % 
% %     TBoot(i,:) = fminsearch(@(tform) runLevMetricBoot( tform, K, scans, images, boot, invert), TOut);
% %     %TBoot(i,:) = cmaes(@(tform) runLevMetricBoot( tform, K, scans, images, boot, invert), TOut, rangeT, opts);
%     TBoot(i,:) = fminsearch(@(tform) runLevMetric( tform, K, scans(i), images(i), invert), TOut);
% end
% TBoot = var(TBoot)';
