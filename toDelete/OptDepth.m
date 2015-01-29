function [ TOut ] = OptDepth(TMean,TVar, RMean, RVar, baseSensor,sensorData, range, imageSensors)

%% get data
numImages = 20;
dataNum = datasample(range,numImages,'Replace',false);

%load images
images = cell(numImages,size(imageSensors,1));

for i = 1:size(imageSensors,1)   
    %previous image
    for j = 1:numImages
        
        left = imread([sensorData{imageSensors(i)}.folder sensorData{imageSensors(i)}.files(dataNum(j)).name]);
        if(size(left,3) == 3)
            left = rgb2gray(left);
        end
        
        right = imread([sensorData{imageSensors(i)}.folder '../../image_01/data/' sensorData{imageSensors(i)}.files(dataNum(j)).name]);
        if(size(right,3) == 3)
            right = rgb2gray(right);
        end
        
        images{j,i} = disparity(left,right);
        images{j,i} = (sensorData{imageSensors(i)}.K(1,1)*0.537)./images{j,i};
        
        images{j,i} = gpuArray(single(images{j,i}));
    end
end

%load cameras
cams = cell(size(imageSensors,1),1);
for i = 1:size(imageSensors,1)
    cams{i} = sensorData{imageSensors(i)}.K;
    cams{i} = gpuArray(single(cams{i}));
end

%load scans
scans = cell(numImages,1);
for i = 1:numImages
    scans{i} = ReadKittiVelDataSingle([sensorData{baseSensor}.folder sensorData{baseSensor}.files(dataNum(i)-1).name]); 
    scans{i}(:,4) = sqrt(sum(scans{i}(:,1:3).^2,2));
    scans{i} = gpuArray(single(scans{i}));
end

%get relavent transforms
TMean = TMean(imageSensors,:); RMean = RMean(imageSensors,:);
idx = [3*imageSensors-2, 2*imageSensors-1, imageSensors];
idx = idx'; idx = idx(:);
TVar = TVar(idx,:); RVar = RVar(idx,:);

TMean = [TMean,RMean];
rangeT = [sqrt(diag(TVar)), sqrt(diag(RVar))];

TMean(1:3) = [0,0,0];
rangeT = [0.5;0.5;0.5;0.2;0.2;0.2];

lb = TMean - repmat(rangeT',size(TMean,1),1);
ub = TMean + repmat(rangeT',size(TMean,1),1);

% TMeanT = TMean(:,1:3);
% lbT = lb(:,1:3);
% ubT = ub(:,1:3);
% 
% %out = fminsearch(@(tform) runFlowMetric( tform, cams, scans, images ), [TMean,RMean]);
% TMean(:,1:3) = simulannealbnd(@(tform) runFlowMetric( tform, TMean, cams, scans, images, 1 ),TMeanT,lbT,ubT);
% 
% TMeanR = TMean(:,4:6);
% lbR = lb(:,4:6);
% ubR = ub(:,4:6);

%TMean= simulannealbnd(@(tform) runFlowMetric( tform, TMean, cams, scans, images, 0 ),TMean,lb,ub)
%TMean= fminsearch(@(tform) runFlowMetric( tform, TMean, cams, scans, images, 0 ),TMean)

% options = psooptimset('PopulationSize', 200,...
%     'TolCon', 1e-1,...
%     'ConstrBoundary', 'absorb',...
%     'StallGenLimit', 50,...
%     'Generations', 300);
% 
% TMean = pso(@(tform) runDepthMetric( tform, TMean, cams, scans, images, 0 ), 6,[],[],[],[],lb,ub,[],options);
TMean= simulannealbnd(@(tform) runDepthMetric( tform, TMean, cams, scans, images, 0 ),TMean,lb,ub)
