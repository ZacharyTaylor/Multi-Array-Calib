% this script generates the first set of results in the paper
% It calculates the transformation between the velodyne and camera 0 for drive
% 28 of the kitti data set using the presented method and a simple equal
% weighted least squares method

%% user set variables

%data range (start excluded as not all sensors running)
range = 50:4000;

%number of scans to use
scansRange = 200:20:2000;

%number of times to perform test
reps = 100;

samples = 10000;

%% setup folders

%contains most of the presentable code
addpath('./finalClean');

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

% %% process velodyne
% load('kittiVelData.mat');
% sensorData{tformIdx,1} = velData;
% tformIdx = tformIdx + 1;
% 
% %% process nav
% load('kittiNavData.mat');
% sensorData{tformIdx,1} = navData;
% tformIdx = tformIdx + 1;

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

T = cell(length(sensorData),1);
t = cell(length(sensorData),1);
for i = 1:length(sensorData)
    [~,idx] = sort(sensorData{i}.time);
    valid = diff(double(sensorData{i}.time(idx))) > 0;
    idx = idx(valid);
    t{i} = double(sensorData{i}.time(idx));
    
    %get absolute angle magnitude
    T{i} = sqrt(sum(sensorData{i}.T_S1_Sk(idx,4:6).^2,2));
end

Res2 = zeros(reps,length(T),length(scansRange));
for w = 1:reps;
    for i = 1:length(scansRange)
        
        %get random index to use for scans
        idx = randi(length(T{1})-scansRange(i)-1)+1;
        idx = [t{1}(idx),t{1}(idx+scansRange(i))]';
        
        offset = [0; 1000000*(2*(rand(length(T)-1,1)-0.5))];
        %offset = zeros(length(T),1);
        
        %get scans
        Ti = cell(length(sensorData),1);
        ti = cell(length(sensorData),1);
        for j = 1:length(T)
            valid = and(t{j} <= idx(2), t{j} >= idx(1));
            Ti{j} = T{j}(valid);
            ti{j} = t{j}(valid) + offset(j);
        end
        
        out = findErr2(Ti,ti,samples) - offset;
        
        Res2(w,:,i) = out';
    end
    w
    save('Test_5_Res2.mat', 'Res2');
end
    
