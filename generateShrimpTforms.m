% this script generates all the required transforms for the kitti and
% shrimp dataset (currently wont do the camera as I am tweaking it)

%% user set variables

%path to data
shrimpPath = '/home/z/Documents/Datasets/Shrimp/high-clutter-2';

%Sets if the sensor transforms will be plotted
plotTforms = false;

%% setup folders

%velodyne processing
addpath('./genShrimpVel/');
addpath('./genShrimpVel/velicp/matlab/');

%nav processing
addpath('./genShrimpNav');

addpath('./handEye');
addpath('./finalClean');

%cam processing
addpath('./genShrimpCam');

%do things in parrallel to save time
for i = 1
    i
    switch i
        case 1
            shrimpVelData = genShrimpVel(shrimpPath, plotTforms, []);
            parsave('shrimpVelData2.mat', shrimpVelData, 'velData');
        case 2
            shrimpCamData = genShrimpCam(shrimpPath, plotTforms, [], 1);
            parsave('shrimpCam1Data.mat', shrimpCamData, 'cam1Data');
        case 3
            shrimpCamData = genShrimpCam(shrimpPath, plotTforms, [], 2);
            parsave('shrimpCam2Data.mat', shrimpCamData, 'cam2Data');
        case 4
            shrimpCamData = genShrimpCam(shrimpPath, plotTforms, [], 3);
            parsave('shrimpCam3Data.mat', shrimpCamData, 'cam3Data');
        case 5
            shrimpCamData = genShrimpCam(shrimpPath, plotTforms, [], 4);
            parsave('shrimpCam4Data.mat', shrimpCamData, 'cam4Data');
        case 6
            shrimpCamData = genShrimpCam(shrimpPath, plotTforms, [], 5);
            parsave('shrimpCam5Data.mat', shrimpCamData, 'cam5Data');
        case 7
            shrimpNavData = genShrimpNav(shrimpPath, plotTforms, []);
            parsave('shrimpNavData.mat', shrimpNavData, 'navData');    
        otherwise
    end
end
    
delete(gcp);