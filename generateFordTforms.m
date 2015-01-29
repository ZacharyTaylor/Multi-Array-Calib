% this script generates all the required transforms for the kitti and
% Ford dataset (currently wont do the camera as I am tweaking it)

%% user set variables

%path to data
FordPath = '/home/z/Documents/Datasets/IJRR-Dataset-1';

%Sets if the sensor transforms will be plotted
plotTforms = false;

%% setup folders

%velodyne processing
addpath('./genFordVel/');
addpath('./genFordVel/velicp/matlab/');

%nav processing
addpath('./genFordNav');

addpath('./handEye');
addpath('./finalClean');

%cam processing
addpath('./genFordCam');

%do things in parrallel to save time
for i = 1
    i
    switch i
        case 1
            FordVelData = genFordVel(FordPath, plotTforms, []);
            parsave('FordVelData.mat', FordVelData, 'velData');
        case 2
            FordCamData = genFordCam(FordPath, plotTforms, [], 1);
            parsave('FordCam1Data.mat', FordCamData, 'cam1Data');
        case 3
            FordCamData = genFordCam(FordPath, plotTforms, [], 2);
            parsave('FordCam2Data.mat', FordCamData, 'cam2Data');
        case 4
            FordCamData = genFordCam(FordPath, plotTforms, [], 3);
            parsave('FordCam3Data.mat', FordCamData, 'cam3Data');
        case 5
            FordCamData = genFordCam(FordPath, plotTforms, [], 4);
            parsave('FordCam4Data.mat', FordCamData, 'cam4Data');
        case 6
            FordCamData = genFordCam(FordPath, plotTforms, [], 5);
            parsave('FordCam5Data.mat', FordCamData, 'cam5Data');
        case 7
            FordNavData = genFordNav(FordPath, plotTforms, []);
            parsave('FordNavData.mat', FordNavData, 'navData');    
        otherwise
    end
end
    
delete(gcp);