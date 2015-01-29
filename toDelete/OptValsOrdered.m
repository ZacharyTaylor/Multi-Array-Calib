function [ TOut ] = OptValsOrdered(TMean,TVar,RMean,RVar,baseSensor,sensorData, range, imageSensors)

%% get data

numImages = length(range);

%load images
images = cell(numImages,size(imageSensors,1));

for i = 1:size(imageSensors,1)   
    %current image
    for j = 1:numImages
        images{j,i} = imread([sensorData{imageSensors(i)}.folder sensorData{imageSensors(i)}.files(range(j)).name]);
        if(size(images{j,i},3) == 3)
            images{j,i} = rgb2gray(images{i,j});
        end
        
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
    scans{i} = ReadKittiVelDataSingle([sensorData{baseSensor}.folder sensorData{baseSensor}.files(range(i)).name]); 
    scans{i}(:,7) = MyHistEq(scans{i}(:,4));
    if(i > 1)
        temp = (sensorData{baseSensor}.T_Skm1_Sk(:,:,range(i))*[scans{i-1}(:,1:3), ones(size(scans{i-1},1),1)]')';
        scans{i-1}(:,4) = knnsearch(scans{i}(:,1:3),temp(:,1:3),'k',1);
    end
    %scans{i}(:,4:6) = temp(:,1:3);
end

for i = 1:numImages
    scans{i} = gpuArray(single(scans{i}));
end

%get relavent transforms
TMean = TMean(imageSensors,:); RMean = RMean(imageSensors,:);
idx = [3*imageSensors-2, 3*imageSensors-1, 3*imageSensors];
idx = idx'; idx = idx(:);
TVar = TVar(idx,:); RVar = RVar(idx,:);

rangeT = [sqrt(diag(TVar)), sqrt(diag(RVar))];

TMean = [TMean, RMean];

lb = TMean - repmat(rangeT',size(TMean,1),1);
ub = TMean + repmat(rangeT',size(TMean,1),1);

TMeanT = TMean(:,1:3);
lbT = lb(:,1:3);
ubT = ub(:,1:3);

% %out = fminsearch(@(tform) runFlowMetric( tform, cams, scans, images ), [TMean,RMean]);
%TMean(:,1:3) = simulannealbnd(@(tform) runFlowMetric( tform, TMean, cams, scans, images, 1 ),TMeanT,lbT,ubT);
% 
% TMeanR = TMean(:,4:6);
% lbR = lb(:,4:6);
% ubR = ub(:,4:6);

%TMean= simulannealbnd(@(tform) runFlowMetric( tform, TMean, cams, scans, images, 0 ),TMean,lb,ub)
TMean= fminsearch(@(tform) runFlowMetric( tform, TMean, cams, scans, images, 0 ),TMean)

% pop = mvnrnd(TMean,TVar,300);
% 
% lb = [-1,-1,-1,-pi,-pi,-pi];
% ub = [1,1,1,pi,pi,pi];
% 
% for i = 1:6
%     pop(pop(:,i) > ub(i)) = ub(i) - 0.01;
%     pop(pop(:,i) < lb(i)) = lb(i) + 0.01;
% end
% 
% options = psooptimset('PopulationSize', 300,...
%     'TolCon', 1e-1,...
%     'ConstrBoundary', 'absorb',...
%     'StallGenLimit', 50,...
%     'Generations', 300,...
%     'PlotFcns', {@psoplotbestf,@psoplotswarmsurf},...
%     'InitialPopulation', pop);
% 
% TMean = pso(@(tform) runFlowMetric( tform, TMean, cams, scans, images, 0 ), 6,[],[],[],[],lb,ub,[],options);

