function [ TVecOut, vTVecOut ] = RefineVelCam( TVecIn,vTVecIn,lidarInfo,camInfo,metric,numScans )
%REFINEVELCAM Summary of this function goes here
%   Detailed explanation goes here

% if(~strcmpi(metric,'Colour'))
%     error('Currently only the colour metric has been implemented');
% end

%get scans to use
%dataIdx = datasample(2:length(camInfo.files),numScans,'Replace',false);

%get angle magnitudes
Mag = sqrt(sum(camInfo.T_Skm1_Sk(:,4:6).^2,2));
[~,idx] = sort(Mag,'descend');
dataIdx = idx(1:numScans);

%disable warning that gets spammed
warning('off','images:initSize:adjustingMag');

%% load images and scans
images = cell(numScans,2);
scans = cell(numScans,1);
for j = 1:numScans
    
    if(strcmpi(metric,'Colour'))
        %previous image
        images{j,1} = ProcessImage(camInfo, dataIdx(j)-1, 3, metric);
        %current image
        images{j,2} = ProcessImage(camInfo, dataIdx(j), 3, metric);
    elseif(strcmpi(metric,'Lev'))
        images{j,1} = ProcessImage(camInfo, dataIdx(j), 5, metric);
    end
    
    %matching lidar scan
    scans{j} = ProcessScan(lidarInfo, camInfo, dataIdx(j), metric);
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
    TVecOut = cmaes(@(tform) RunColourMetric(tform, K, scans, images, 2), m,sd, opts);
    %TVecOut = fminsearch(@(tform) RunColourMetric(tform, K, scans, images,2), TVecOut);
elseif(strcmpi(metric,'Lev'))
    TVecOut = cmaes(@(tform) RunLevMetric(tform, K, scans, images, 2), m,sd, opts);
    %TVecOut = fminsearch(@(tform) RunLevMetric(tform, K, scans, images,2), TVecOut);
end
TVecOut = TVecOut';

if(TVecOut(1:3) > 3)
    warning('Alignment failed');
    TVecOut = zeros(1,6);
    vTVecOut = inf*ones(1,6);
    return;
end

v = zeros(numScans,6);
step = 0.01;

dxx = zeros(6,6);
for i = 1:6
    for j = 1:6
        temp = TVecOut';
        temp(i) = temp(i) + step;
        temp(j) = temp(j) + step;
        f1 = RunColourMetric(temp, K, scans, images,2);

        temp = TVecOut';
        temp(i) = temp(i) - step;
        temp(j) = temp(j) + step;
        f2 = RunColourMetric(temp, K, scans, images,2);
        
        temp = TVecOut';
        temp(i) = temp(i) + step;
        temp(j) = temp(j) - step;
        f3 = RunColourMetric(temp, K, scans, images,2);
        
        temp = TVecOut';
        temp(i) = temp(i) - step;
        temp(j) = temp(j) - step;
        f4 = RunColourMetric(temp, K, scans, images,2);

        dxx(i,j) = (f1-f2-f3+f4)/(4*step*step);
    end
end

dxz = zeros(6,6*numScans);
for i = 1:6
    temp = zeros(numScans,6);
    for j = 1:numScans
        for k = 1:6
            tempA = TVecOut';
            tempA(i) = tempA(i) + step;
            offset = zeros(6,1);
            offset(k) = offset(k) + step;
            tempB = scans;
            [tempB{j},v(j,:)] = OffsetScan(tempB{j}, lidarInfo, camInfo, dataIdx(j),offset');
            f1 = RunColourMetric(tempA, K, tempB(j,:), images(j,:),2);
        
            tempA = TVecOut';
            tempA(i) = tempA(i) - step;
            offset = zeros(6,1);
            offset(k) = offset(k) + step;
            tempB = scans;
            [tempB{j},v(j,:)] = OffsetScan(tempB{j}, lidarInfo, camInfo, dataIdx(j),offset');
            f2 = RunColourMetric(tempA, K, tempB(j), images(j,:),2);
            
            tempA = TVecOut';
            tempA(i) = tempA(i) + step;
            offset = zeros(6,1);
            offset(k) = offset(k) - step;
            tempB = scans;
            [tempB{j},v(j,:)] = OffsetScan(tempB{j}, lidarInfo, camInfo, dataIdx(j),offset');
            f3 = RunColourMetric(tempA, K, tempB(j,:), images(j,:),2);
            
            tempA = TVecOut';
            tempA(i) = tempA(i) - step;
            offset = zeros(6,1);
            offset(k) = offset(k) - step;
            tempB = scans;
            [tempB{j},v(j,:)] = OffsetScan(tempB{j}, lidarInfo, camInfo, dataIdx(j),offset');
            f4 = RunColourMetric(tempA, K, tempB(j,:), images(j,:),2);
            
            temp(j,k) = (f1-f2-f3+f4)/(4*step*step);
        end
    end
    dxz(i,:) = temp(:);
end

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

    %blur image
    G = fspecial('gaussian',[50 50],blur);
    image = imfilter(image,G,'same');
    
    image = imgradient(image);
   
    %undistort image
    image = Undistort(double(image), camInfo.D, camInfo.K);
    
    %histogram equalize
    image = image - min(image(:));
    image = image ./ max(image(:));
    image = 255*image;
    %image = 255*MyHistEq(image);
    
    %mask image
    if(isfield(camInfo,'mask'))
        image(~camInfo.mask) = 0;
    end
    
    %move to gpu
    image = gpuArray(uint8(image));
end

function [ scan ] = ProcessScan(lidarInfo, camInfo, idx, metric)

    if(strcmpi(metric,'Colour'))
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
        %tform = inv(tform);

        %get difference in points
        dPoints = (tform*[points, ones(size(points,1),1)]')';
        dPoints = dPoints(:,1:3) - points;

        %interpolate point positions
        tStartFrac = (double(camInfo.time(idx-1))-TPrev)/(tCurr - TPrev);
        tEndFrac = (double(camInfo.time(idx))-TPrev)/(tCurr - TPrev);
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
    [tform, tformV] = IndVar(0.01,findT,lidarInfo.T_S1_Sk(idxPrev,:),zeros(1,6),lidarInfo.T_S1_Sk(idxCurr,:),(lidarInfo.T_Var_S1_Sk(idxCurr,:)-lidarInfo.T_Var_S1_Sk(idxPrev,:)));
    
    %get final tform with offset
    tform = tform + offset;
    tform = V2T(tform);
    points = double(gather(scanIn(:,1:3)));
    
    %get difference in points
    dPoints = (tform*[points, ones(size(points,1),1)]')';
    dPoints = dPoints(:,1:3) - points;
    
    %interpolate point positions
    tStartFrac = (double(camInfo.time(idx-1))-TPrev)/(tCurr - TPrev);
    tEndFrac = (double(camInfo.time(idx))-TPrev)/(tCurr - TPrev);
    %tStartFrac = 0;
    %tEndFrac = 1;
    
    %combine
    points = [points + tStartFrac*dPoints, points + tEndFrac*dPoints];
    
    %move everything to the gpu
    scanOut = gpuArray(single([points, scanIn(:,7:end)]));
end

