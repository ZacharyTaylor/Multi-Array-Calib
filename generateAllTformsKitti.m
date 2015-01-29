% this script generates all the required transforms for the kitti and
% shrimp dataset (currently wont do the camera as I am tweaking it)

%% user set variables

%path to data
%kittiPath = 'D:\2011_10_03_drive_0027_extract\';
kittiPath = '/home/z/Documents/Datasets/Kitti/2011_10_03_drive_0027_extract/';

%Sets if the sensor transforms will be plotted
plotTforms = false;

%% setup folders

%velodyne processing
addpath('./genKittiVel/');
addpath('./genKittiVel/velicp/matlab/');

%nav processing
addpath('./genKittiNav');

%cam processing
addpath('./genKittiCam');

addpath('./handEye');

%do things in parrallel to save time
parfor i = 2:5
    switch i
        case 1
            kittiVelData = genKittiVel(kittiPath, plotTforms, []);
            %parsave('kittiVelData2.mat', kittiVelData, 'velData');
        case 2
            kittiCamData = genKittiCam(kittiPath, plotTforms, [], 2);
            parsave('kittiCam2Data2.mat', kittiCamData, 'cam2Data');
        case 3
            kittiCamData = genKittiCam(kittiPath, plotTforms, [], 3);
            parsave('kittiCam3Data2.mat', kittiCamData, 'cam3Data');
        case 4
            kittiCamData = genKittiCam(kittiPath, plotTforms, [], 4);
            parsave('kittiCam4Data2.mat', kittiCamData, 'cam4Data');
        case 5
            kittiCamData = genKittiCam(kittiPath, plotTforms, [], 1);
            parsave('kittiCam1Data2.mat', kittiCamData, 'cam1Data');
        case 6
            kittiNavData = genKittiNav(kittiPath, plotTforms, []);
            parsave('kittiNavData.mat', kittiNavData, 'navData');    
        otherwise
    end
end
    
delete(gcp);