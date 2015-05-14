function [ TVecOut, vTVecOut ] = RefineVelCam( TVecIn,vTVecIn,lidarInfo,camInfo,metric,numScans )
%REFINEVELCAM Summary of this function goes here
%   Detailed explanation goes here

sep = 1;

% if(~strcmpi(metric,'Colour'))
%     error('Currently only the colour metric has been implemented');
% end

%get scans to use
dataIdx = datasample(3:length(camInfo.files),numScans,'Replace',false);

%get angle magnitudes
mag = sqrt(sum(camInfo.T_Skm1_Sk(:,4:6).^2,2));

[~,idx] = sort(mag,'descend');
mag(idx(ceil(0.75*length(mag)):end)) = 0;

%divide into sections
step = floor(length(mag)/numScans);
for i = 1:numScans
    val = mag;
    val(1) = 0;
    val(1:step*(i-1)) = 0;
    val(step*i:end) = 0;
    [~,idx] = max(val);
    dataIdx(i) = idx;
end

% [~,idx] = sort(mag,'descend');
% dataIdx = idx(1:numScans);

%disable warning that gets spammed
warning('off','images:initSize:adjustingMag');

%% load images and scans
images = cell(numScans,2);
scans = cell(numScans,1);

for j = 1:numScans
    if(strcmpi(metric,'Colour'))
        %previous image
        images{j,1} = ProcessImage(camInfo, dataIdx(j)-sep, false, metric);
        %current image
        images{j,2} = ProcessImage(camInfo, dataIdx(j), false, metric);
    end
    
    %matching lidar scan
    scans{j} = ProcessScan(lidarInfo, camInfo, dataIdx(j), metric, sep);
end

%load camera
K = camInfo.K;
K = gpuArray(single(K));

%setup inital mean and sd
sd = sqrt(vTVecIn)';
m = TVecIn';

%set up optimizer
opts.LBounds = [-3.5,-3.5,-3.5,-pi/2,-pi/2,-pi]'; opts.UBounds = [3.5,3.5,3.5,pi/2,pi/2,pi]'; 
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
if(strcmpi(metric,'Colour'))
   TVecOut = cmaes(@(tform) RunColourMetric(tform, K, scans, images, 2, true), m,sd, opts);
elseif(strcmpi(metric,'Lev'))
    TVecOut = cmaes(@(tform) RunLevMetric(tform, K, scans, images, 2, true), m,sd, opts);
end
TVecOut = TVecOut';

if(any(abs(TVecOut(1:3)) > 3))
    warning('Alignment failed');
    TVecOut = zeros(1,6);
    vTVecOut = inf*ones(6,1);
    return;
end

for j = 1:numScans
    if(strcmpi(metric,'Colour'))
        %previous image
        images{j,1} = ProcessImage(camInfo, dataIdx(j)-1, true, metric);
        %current image
        images{j,2} = ProcessImage(camInfo, dataIdx(j), true, metric);
    end
end

vTVecOut = FindVar(numScans,TVecOut,scans,images, lidarInfo, camInfo, K, dataIdx, [0.0001,0.0001,0.0001,0.00001,0.00001,0.00001]);

end

function [vTVecOut] = FindVar(numScans,TVecOut,scans,images, lidarInfo, camInfo, K, dataIdx, step)
    v = zeros(numScans,6);

    %chop edges off scans
    T = inv(V2T(TVecOut));
    T = gpuArray(single(T));
    ims = size(images{1});
    
    mc = camInfo.mask;
    mc(1,:) = false; mc(end,:) = false; mc(:,1) = false; mc(:,end) = false;
    mc = gpuArray(single(imerode(mc,strel('disk',31))));
    for i = 1:size(scans,1)
        [pro1,valid1] = projectLidar(T, K, scans{i}(:,1:3), ims(1:2));
        valid1(valid1) = interpolateImage(mc, pro1(valid1,1:2));
        [pro2,valid2] = projectLidar(T, K, scans{i}(:,4:6), ims(1:2));
        valid2(valid2) = interpolateImage(mc, pro2(valid2,1:2));

        scans{i} = scans{i}(and(valid1,valid2),:);
    end
    
    dxx = zeros(6,6);
    for i = 1:6
        for j = 1:6
            temp = TVecOut';
            temp(i) = temp(i) + step(i);
            temp(j) = temp(j) + step(j);
            f1 = RunColourMetric(temp, K, scans, images,2,false);

            temp = TVecOut';
            temp(i) = temp(i) - step(i);
            temp(j) = temp(j) + step(j);
            f2 = RunColourMetric(temp, K, scans, images,2,false);

            temp = TVecOut';
            temp(i) = temp(i) + step(i);
            temp(j) = temp(j) - step(j);
            f3 = RunColourMetric(temp, K, scans, images,2,false);

            temp = TVecOut';
            temp(i) = temp(i) - step(i);
            temp(j) = temp(j) - step(j);
            f4 = RunColourMetric(temp, K, scans, images,2,false);

            dxx(i,j) = (f1-f2-f3+f4)/(4*step(i)*step(j));
        end
    end

    dxz = zeros(6,6*numScans);
    for i = 1:6
        temp = zeros(numScans,6);
        for j = 1:numScans
            for k = 1:6
                tempA = TVecOut';
                tempA(i) = tempA(i) + step(i);
                offset = zeros(6,1);
                offset(k) = offset(k) + step(k);
                tempB = scans;
                [tempB{j},v(j,:)] = OffsetScan(tempB{j}, lidarInfo, camInfo, dataIdx(j),offset');
                f1 = RunColourMetric(tempA, K, tempB(j,:), images(j,:),2,false);

                tempA = TVecOut';
                tempA(i) = tempA(i) - step(i);
                offset = zeros(6,1);
                offset(k) = offset(k) + step(k);
                tempB = scans;
                [tempB{j},v(j,:)] = OffsetScan(tempB{j}, lidarInfo, camInfo, dataIdx(j),offset');
                f2 = RunColourMetric(tempA, K, tempB(j,:), images(j,:),2,false);

                tempA = TVecOut';
                tempA(i) = tempA(i) + step(i);
                offset = zeros(6,1);
                offset(k) = offset(k) - step(k);
                tempB = scans;
                [tempB{j},v(j,:)] = OffsetScan(tempB{j}, lidarInfo, camInfo, dataIdx(j),offset');
                f3 = RunColourMetric(tempA, K, tempB(j,:), images(j,:),2,false);

                tempA = TVecOut';
                tempA(i) = tempA(i) - step(i);
                offset = zeros(6,1);
                offset(k) = offset(k) - step(k);
                tempB = scans;
                [tempB{j},v(j,:)] = OffsetScan(tempB{j}, lidarInfo, camInfo, dataIdx(j),offset');
                f4 = RunColourMetric(tempA, K, tempB(j,:), images(j,:),2,false);

                temp(j,k) = (f1-f2-f3+f4)/(4*step(i)*step(k));
            end
        end
        dxz(i,:) = temp(:);
    end
    
    %correct for overconfident variance predictions
    [~,idx] = sort(v);
    v(idx(1:floor(size(idx,1)*0.25)),:) = repmat(v(idx(floor(size(idx,1)*0.25)+1),:),floor(size(idx,1)*0.25),1);

    d = dxx\dxz;
    d = (d.*repmat(v(:)',size(d,1),1))*d';
    vTVecOut = diag(d);
end

function [ image ] = ProcessImage(camInfo, idx, blur, metric)

    %load image
    image = imread([camInfo.folder camInfo.files(idx).name]);
    if(size(image,3) == 3)
        image = rgb2gray(image);
    end

    image = double(image);
    
    %undistort image
    image = Undistort(double(image), camInfo.D, camInfo.K);
    
    %image = imgradient(image);
    %image = image + 0.000001;
    
    if blur
        b = max(size(image))/200;
        G = fspecial('gaussian',[100 100],b);
    
        image = imfilter(image,G,'same');
    end
    
    %histogram equalize
    %image = image - min(image(:));
    %image = image ./ max(image(:));
    %image = 255*image;
    %image = 255*MyHistEq(image);
    
    %mask image
    if(isfield(camInfo,'mask'))
        image(~camInfo.mask) = 0;
    end
    
    %move to gpu
    image = gpuArray(single(image));
end

function [ scan ] = ProcessScan(lidarInfo, camInfo, idx, metric,sep)

    if(strcmpi(metric,'Colour'))
        %match lidar with previous camera frame
        time = double(lidarInfo.time) - double(camInfo.time(idx-sep));
        [~,idxPrev] = min(abs(time));
        tPrev = double(lidarInfo.time(idxPrev));

        %read lidar data
        [xyz, intensity, timeFrac] = ReadVelData([lidarInfo.folder lidarInfo.files(idxPrev).name]);
        
        %eliminiate moving points
        xyzNext = ReadVelData([lidarInfo.folder lidarInfo.files(idxPrev+1).name]);
        xyzNext = V2T(lidarInfo.T_Skm1_Sk(idxPrev+1,:))\[xyzNext, ones(size(xyzNext,1),1)]';
        xyzNext = xyzNext(1:3,:)';

        i = knnsearch(xyzNext,xyz);
        xyzNext = xyzNext(i,:);

        i = knnsearch(xyz,xyz,'k',3);
        n = cross(xyz(i(:,2),:)-xyz(i(:,1),:),xyz(i(:,3),:)-xyz(i(:,1),:));
        n = n./repmat(sqrt(sum(n.^2,2)),1,3);
        n(isnan(n)) = 0;

        err = sqrt(sum((n.*(xyzNext - xyz)).^2,2));
        valid = (err < 0.1);

        xyz = xyz(valid,:);
        intensity = intensity(valid,:);
        timeFrac = timeFrac(valid,:);
        
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
        %tform = inv(tform);

        %get difference in points
        dPoints = (tform*[points, ones(size(points,1),1)]')';
        dPoints = dPoints(:,1:3) - points;

        %interpolate point positions
        tStartFrac = (double(camInfo.time(idx-sep))-tPrev)/(tCurr - tPrev);
        tEndFrac = (double(camInfo.time(idx))-tPrev)/(tCurr - tPrev);
        
        %tStartFrac = 0;
        %tEndFrac = 1;
        
        %combine
        points = [points + tStartFrac*dPoints, points + tEndFrac*dPoints];

        %convert intensity to colours
        cmap = colormap(jet(256));
        intensity = cmap(round(255*intensity)+1,:);
        
        points = [points, intensity];
        
        %remove points that may be occluded
        points = OcCut(points);
        
        %move everything to the gpu
        scan = gpuArray(single(points));
    end
    
    if(strcmpi(metric,'Lev'))
        %match lidar with previous camera frame
        time = double(lidarInfo.time) - double(camInfo.time(idx));
        [~,idx] = min(abs(time));

        %read lidar data
        [xyz, intensity, timeFrac] = ReadVelData([lidarInfo.folder lidarInfo.files(idx).name]);

        points = xyz;
        points = LevSub(points);

        %convert intensity to colours
        cmap = colormap(jet(256));
        intensity = cmap(round(255*intensity)+1,:);

        %move everything to the gpu
        scan = gpuArray(single([points, intensity]));
    end
end

function [ scanOut, tformV ] = OffsetScan(scanIn, lidarInfo, camInfo, idx,offset)

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
    [~, tformV] = IndVar(0.001,findT,lidarInfo.T_S1_Sk(idxPrev,:),zeros(1,6),lidarInfo.T_S1_Sk(idxCurr,:),(lidarInfo.T_Var_S1_Sk(idxCurr,:)-lidarInfo.T_Var_S1_Sk(idxPrev,:)));
    
    %get final tform with offset
    tform = offset;
    tform = V2T(tform);
    points = scanIn(:,4:6);
    
    %get difference in points
    dPoints = (tform*[points, ones(size(points,1),1)]')';
    scanIn(:,4:6) = dPoints(:,1:3);
    
    scanOut = scanIn;
end

