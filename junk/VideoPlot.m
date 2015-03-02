%data range (start excluded as not all sensors running)
range = 50:4000;

%% setup folders

%contains most of the presentable code
addpath('./finalClean');

addpath('./exportFig');

addpath('./tformInterp');
addpath('./imageMetric');

addpath('./genKittiCam');

%hand eye calibration
addpath('./handEye/');

%% clear previous data
tformIdx = 1;
clear tform;
clear tformVar;
clear sensorType;
clear sensorData;

%% process velodyne
load('kittiVelData.mat');
sensorData{tformIdx,1} = velData;
tformIdx = tformIdx + 1;

%% process nav
load('kittiNavData.mat');
sensorData{tformIdx,1} = navData;
tformIdx = tformIdx + 1;

%% process cameras
load('kittiCam1Data.mat');
sensorData{tformIdx,1} = cam1Data;
tformIdx = tformIdx + 1;

load('kittiCam2Data.mat');
sensorData{tformIdx,1} = cam2Data;
tformIdx = tformIdx + 1;

load('kittiCam3Data.mat');
sensorData{tformIdx,1} = cam3Data;
tformIdx = tformIdx + 1;

load('kittiCam4Data.mat');
sensorData{tformIdx,1} = cam4Data;
tformIdx = tformIdx + 1;

%% find transformations

for i = 1:length(sensorData)
    if(i > 1)
        sensorData{i} = matchTforms(sensorData{i}, sensorData{1},range, false);
    else
        sensorData{i}.T_Skm1_Sk = sensorData{i}.T_Skm1_Sk(range,:);
        sensorData{i}.T_S1_Sk = sensorData{i}.T_S1_Sk(range,:);
        sensorData{i}.T_Cov_Skm1_Sk = sensorData{i}.T_Cov_Skm1_Sk(range,:);
        sensorData{i}.time = sensorData{i}.time(range,:);
        sensorData{i}.files = sensorData{i}.files(range,:);
    end
end

T = cell(size(sensorData,1),2);
for i = 1:length(sensorData)
    T{i,1} = eye(4);
    T{i,2} = eye(4);
end

out = zeros(3000,9);

for j = 1:3000
    j
    T{1,1} = T{1,1}*inv(vec2tran(sensorData{1,1}.T_Skm1_Sk(j,:)'));

    T{2,1} = T{2,1}*inv(vec2tran(sensorData{2,1}.T_Skm1_Sk(j,:)'));

    sensorData{3,1}.T_Skm1_Sk(j,1:3) = sensorData{3,1}.T_Skm1_Sk(j,1:3)*norm(sensorData{1,1}.T_Skm1_Sk(j,1:3));
    T{3,1} = T{3,1}*inv(vec2tran(sensorData{3,1}.T_Skm1_Sk(j,:)'));

    out(j,:) = [T{1}(1:3,4)',T{2}(1:3,4)',T{3}(1:3,4)'];
end

dlmwrite('out.csv',out);
