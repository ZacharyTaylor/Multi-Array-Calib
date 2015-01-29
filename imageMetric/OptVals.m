function [ TOut, TBoot ] = OptVals(TMean,TVar, lidar, cam, invert)

%% get data
matchNum = 40;
depth = 1;
dataNum = datasample((1+depth):(size(cam.files,1)),matchNum,'Replace',false);

warning('off','images:initSize:adjustingMag');

%load images
images = cell(matchNum,4);

G = fspecial('gaussian',[50 50],1);

%previous image
for j = 1:matchNum
    images{j,1} = imread([cam.folder cam.files(dataNum(j)-depth).name]);
    if(size(images{j,1},3) == 3)
        images{j,1} = rgb2gray(images{j,1});
    end
    
    images{j,1} = imfilter(images{j,1},G,'same');
    
    if(isfield(cam,'mask'))
        images{j,1}(~cam.mask) = 0;
    end
    
    images{j,1} = undistort(double(images{j,1}), cam.D, cam.K(1:3,1:3));
    
    [x,y] = gradient(images{j,1});
    images{j,3} = sqrt(x.^2 + y.^2);
    images{j,3} = images{j,3} - min(images{j,3}(:));
    images{j,3} = uint8(254*(images{j,3} ./ max(images{j,3}(:)))+1);
    images{j,3} = gpuArray(images{j,3});
    
    images{j,1} = gpuArray(uint8(images{j,1}));
end

for j = 1:matchNum
    images{j,2} = imread([cam.folder cam.files(dataNum(j)).name]);
    if(size(images{j,2},3) == 3)
        images{j,2} = rgb2gray(images{j,2});
    end
    
    images{j,2} = imfilter(images{j,2},G,'same');
    
    if(isfield(cam,'mask'))
        images{j,2}(~cam.mask) = 0;
    end
    
    images{j,2} = undistort(double(images{j,2}), cam.D, cam.K(1:3,1:3));
    
    [x,y] = gradient(images{j,2});
    images{j,4} = sqrt(x.^2 + y.^2);
    images{j,4} = images{j,4} - min(images{j,4}(:));
    images{j,4} = uint8(254*(images{j,4} ./ max(images{j,4}(:)))+1);
    images{j,4} = gpuArray(images{j,4});
   
    images{j,2} = gpuArray(uint8(images{j,2}));
end

%load camera
K = cam.K;
K = gpuArray(single(K));

%load scans
scans = cell(matchNum,1);
cmap = colormap(jet(256));
for i = 1:matchNum
    
    time = double(lidar.time) - double(cam.time(dataNum(i)-depth));
    [~,idxPrev] = min(abs(time));
    tPrev = double(lidar.time(idxPrev));
    
    if(strcmp(lidar.files(idxPrev).name(end-2:end),'ply'))
        temp = ply_read ([lidar.folder lidar.files(idxPrev).name]);
        temp = [temp.vertex.x, temp.vertex.y, temp.vertex.z, temp.vertex.intensity/255, temp.vertex.valid];
        temp = temp(temp(:,5)>0,1:4);
    else
        temp = dlmread([lidar.folder lidar.files(idxPrev).name],' ');
    end
    
    time = double(lidar.time) - double(cam.time(dataNum(i)));
    [~,idxCurr] = min(abs(time));
    tCurr = double(lidar.time(idxCurr));
    tDiff = idxCurr-idxPrev;
    
    tform = vec2tran(lidar.T_S1_Sk(idxPrev,:)')\vec2tran(lidar.T_S1_Sk(idxCurr,:)');
    
    %offset = -(tPrev-double(cam.time(dataNum(i)-depth)))/(tCurr - tPrev);
    offset = 0;
    
    %temp(:,1:3) = tformInterp3(tform,zeros(size(temp,1),1),temp(:,1:3));
    temp(:,1:3) = tformInterp3(inv(tform),(offset + (getTime(temp, 0)/tDiff)),temp(:,1:3));
    
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
%TMean(1:3) = 0;
rangeT(rangeT > 0.7*opts.UBounds) = 0.7*opts.UBounds(rangeT > 0.7*opts.UBounds);

TOut = cmaes(@(tform) runFlowMetric( tform, K, scans, images, invert ), TMean, rangeT, opts);

TOut = TOut(:);

TBoot = zeros(10,6);

bootMax = 0;
for i = 1:size(scans,1)
    bootMax = max(bootMax,size(scans{i},1));
end

for i = 1:10
    i
    boot = datasample(1:bootMax,bootMax);
    boot = sort(boot);
    [b,idx] = unique(boot);
    idx = [idx;bootMax+1];
    boot = zeros(bootMax,1);
    boot(b) = diff(idx);

    opts.MaxIter = 100;
    TBoot(i,:) = fminsearch(@(tform) runFlowMetricBoot( tform, K, scans, images, boot, invert), TOut);%, rangeT, opts);
end
TBoot = var(TBoot)';

% for i = 1:matchNum
%     opts.MaxIter = 100;
%     TBoot(i,:) = cmaes(@(tform) runFlowMetric( tform, K, scans(i,:), images(i,:), invert ), TOut, rangeT, opts);
%     %TBoot(i,:) = fminsearch(@(tform) runFlowMetric( tform, K, scans(i,:), images(i,:), invert), TOut);
% end
% TBoot = var(TBoot)';


