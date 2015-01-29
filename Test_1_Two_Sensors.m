% this script generates the first set of results in the paper
% It calculates the transformation between the velodyne and camera 0 for drive
% 28 of the kitti data set using the presented method and a simple equal
% weighted least squares method

%% user set variables

%number of scans to use
scansTimeRange = 100;%5:5:100;

%number of times to perform test
reps = 10;

%number of bootstrap iterations to perform
bootNum = 10;

%% setup folders

%contains most of the presentable code
addpath('./finalClean');

addpath('./tformInterp');
addpath('./imageMetric');

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

%% process cameras
load('kittiCam1Data2.mat');
sensorData{tformIdx,1} = cam1Data;
tformIdx = tformIdx + 1;

load('kittiCam2Data2.mat');
sensorData{tformIdx,1} = cam2Data;
tformIdx = tformIdx + 1;
% 
% load('kittiCam3Data2.mat');
% sensorData{tformIdx} = cam3Data;
% tformIdx = tformIdx + 1;
% 
% load('kittiCam4Data2.mat');
% sensorData{tformIdx} = cam4Data;
% tformIdx = tformIdx + 1;

%% process nav
load('kittiNavData.mat');
sensorData{tformIdx,1} = navData;
tformIdx = tformIdx + 1;

%% find transformations

sensorData = addOffset(sensorData, 4000);

RErr = zeros(reps,3,size(scansTimeRange(:),1));
TErr = zeros(reps,3,size(scansTimeRange(:),1));

RVar = zeros(reps,3,size(scansTimeRange(:),1));
TVar = zeros(reps,3,size(scansTimeRange(:),1));

RErrEqual = zeros(reps,3,size(scansTimeRange(:),1));
TErrEqual = zeros(reps,3,size(scansTimeRange(:),1));

for w = 1:reps
    for s = 1:size(scansTimeRange(:),1)
        %get random contiguous scans to use
        sData = randTformsTime(sensorData, scansTimeRange(s));

        %Create equal weighted variance
        sDataE = sData;
        for i = 1:size(sData,1)
            sDataE{i}.T_Cov_Skm1_Sk = ones(size(sData{1}.T_Cov_Skm1_Sk));
        end

        sData = rejectPoints(sData);
        
        %find equal weighted results
        rotVec = roughR(sDataE);
        tranVec = roughT(sDataE, rotVec);

        %write out results
        RErrEqual(w,:,s) = rotVec(2,:);
        TErrEqual(w,:,s) = tranVec(2,:);

        %find rotation
        sData = rejectPoints(sData);
        rotVec = roughR(sData);
        [rotVec, rotVar] = optR(sData, rotVec);
        
        %find translation
        tranVec = roughT(sData, rotVec);
        [tranVec, tranVar] = optT(sData, tranVec, rotVec);

        %bootstrap
        %[tranVar, rotVar] = bootTform(sData, tranVec, rotVec, bootNum);

        %write out results
        RErr(w,:,s) = rotVec(2,:);
        TErr(w,:,s) = tranVec(2,:);
        RVar(w,:,s) = rotVar(2,:);
        TVar(w,:,s) = tranVar(2,:);

        fprintf('R = [% 1.3f,% 1.3f,% 1.3f], T = [% 3.2f,% 3.2f,% 3.2f] using %4i seconds of data, iteration = %i\n',rotVec(2,1),rotVec(2,2),rotVec(2,3),tranVec(2,1),tranVec(2,2),tranVec(2,3),scansTimeRange(s),w);

        save('Test_1_Res.mat', 'RErr', 'TErr', 'RVar', 'TVar', 'RErrEqual', 'TErrEqual');
    end
end
