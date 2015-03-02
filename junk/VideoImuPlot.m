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

T = eye(4);
Told = eye(4);

out = zeros(3000,9);

writerObj = VideoWriter('imuDir.avi');
open(writerObj);

for j = 1:3000
    T = T*inv(vec2tran(sensorData{2,1}.T_Skm1_Sk(j,:)'));
    vec = T(1:3,4)-Told(1:3,4);
    Told = T;
    
    quiver3(0,0,0,vec(1),vec(2),vec(3));
    axis([-1 1 -1 1 -1 1 0 1]);
    drawnow;
    
    frame = getframe;
    writeVideo(writerObj,frame);
    
end

close(writerObj);