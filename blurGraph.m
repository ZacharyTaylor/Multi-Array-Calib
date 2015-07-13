function [] = blurGraph()

numScans = 1;
gt = [0.291804720076481,-0.0114055066485929,-0.0562394126383353,-1.21996291041592,1.19352064861705,-1.20328698135780];
res = 101;
range = 10;

dataIdx = 2000;%[20:50:500];
metric = 'gom';

sensorData = LoadSensorData('Kitti','Vel','Cam1');
lidarInfo = sensorData{1};
camInfo = sensorData{2};

%% load images and scans
images = cell(numScans,1);
scans = cell(numScans,1);

for j = 1:numScans
        images{j,1} = ProcessImage(camInfo, dataIdx(j)-1, false, metric);       
        %matching lidar scan
        scans{j} = ProcessScan(lidarInfo, camInfo, dataIdx(j), metric, 1, V2T(gt));
end

%load camera
K = camInfo.K;
K = gpuArray(single(K));

out = zeros(range,range);
outY = zeros(range,range);
outP = zeros(range,range);
T = V2T(gt);

for i = 1:res
    for j = 1:res
        [r,p,y] = dcm2angle(T(1:3,1:3));
        outR(i,j) = pi*range*(i - ceil(res/2))/(180*res);
        outP(i,j) = pi*range*(j - ceil(res/2))/(180*res);
        r = r + outR(i,j);
        p = p + outP(i,j);
        tform = T;
        tform(1:3,1:3) = angle2dcm(r,p,y);
        out(i,j) = RunGomMetric(T2V(tform)', K, scans, images, 1);
    end
end

x = outR;
y = outP;
z = -out;

a = 1

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
    
    if(strcmpi('motion',metric))
        image = imgradient(image);
        image = image + 0.000001;
    elseif(strcmpi('nmi',metric))
        image = image - min(image(:));
        image = image ./ max(image(:));
    elseif(strcmpi('gom',metric))
        [x,y] = gradient(double(image));
        image(:,:,1) = x/255;
        image(:,:,2) = y/255;
    elseif(strcmpi('lev',metric))
        image = LevImage(image);
    end
    
    if blur
        b = max(size(image))/500;
        G = fspecial('gaussian',[100 100],b);
    
        image = imfilter(image,G,'same');
    end
    
    %mask image
    if(isfield(camInfo,'mask'))
        image(~camInfo.mask) = 0;
    end
    
    %move to gpu
    image = gpuArray(single(image));
end

function [ scan ] = ProcessScan(lidarInfo, camInfo, idx, metric,sep, tInital)

    %match lidar with previous camera frame
    time = double(lidarInfo.time) - double(camInfo.time(idx-sep));
    [~,idxPrev] = min(abs(time));
    tPrev = double(lidarInfo.time(idxPrev));

    %read lidar data
    [xyz, intensity, timeFrac] = ReadVelData([lidarInfo.folder lidarInfo.files(idxPrev).name]);

    %find time difference between scans
    time = double(lidarInfo.time) - double(camInfo.time(idx));
    [~,idxCurr] = min(abs(time));
    tCurr = double(lidarInfo.time(idxCurr));

    %correct for motion during scan
    points = VelCorrect(xyz, timeFrac, lidarInfo.T_Skm1_Sk(idxPrev,:));

    %find transformation between scans
    tform = V2T(lidarInfo.T_S1_Sk(idxPrev,:))\V2T(lidarInfo.T_S1_Sk(idxCurr,:));

    %get difference in points
    dPoints = (tform*[points, ones(size(points,1),1)]')';
    dPoints = dPoints(:,1:3) - points;

    %interpolate point positions
    tStartFrac = (double(camInfo.time(idx-sep))-tPrev)/(tCurr - tPrev);
    tEndFrac = (double(camInfo.time(idx))-tPrev)/(tCurr - tPrev);

    %combine
    points = [points + tStartFrac*dPoints, points + tEndFrac*dPoints];

    %convert intensity to colours
    cmap = colormap(jet(256));
    col = cmap(round(255*intensity)+1,:);

    points = [points, col];

    if(strcmpi('motion',metric))
        %remove points that may be occluded
        points = OcCut(points);
    elseif(strcmpi('nmi',metric))
        points(:,4) = intensity;
        points(:,4) = points(:,4) - min(points(:,4));
        points(:,4) = points(:,4) ./ max(points(:,4));
        points(:,5:6) = 0;
    elseif(strcmpi('gom',metric))
        points(:,4) = intensity;
        points(:,5:6) = 0;
        [points(:,4),points(:,5)] = Get2DGradProject(points(:,1:4), inv(tInital));
        points(:,6) = 0;
        tmp = sqrt(points(:,4).^2 + points(:,5).^2);
        points(:,7:9) = cmap(round(255*(MyHistEq(tmp))+1),:);
    elseif(strcmpi('lev',metric))
        [tmp, valid] = LevLidar(points(:,1:3));
        points = points(valid,:);
        points(:,1:4) = tmp;
        points(:,5:6) = 0;
        points(:,7:9) = cmap(round(255*(tmp(:,4)./max(tmp(:,4)))+1),:);
    end

    %move everything to the gpu
    scan = gpuArray(single(points));
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