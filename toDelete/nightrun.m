% %% user set variables
% 
%path to data
kittiPath = '/home/z/Documents/Datasets/Kitti/2011_10_03_drive_0027_extract';

%Sets if the sensor transforms will be plotted
plotTforms = false;

%% setup folders

%velodyne processing
addpath('./genKittiVel/');
addpath('./genKittiVel/libicp/matlab/');

%nav processing
addpath('./genKittiNav');

%cam processing
addpath('./genKittiCamSBA');

addpath('./tformInterp');
% 
% % kittiVelData = genKittiVel(kittiPath, plotTforms, []);
% % parsave('kittiVelData.mat', kittiVelData, 'velData');
% % 
temp = load('kittiVelData.mat');
camData.time = ReadKittiTimestamps([kittiPath '/image_00/']);
temp.velData = matchTforms(temp.velData, camData,[], true);
scale = sqrt(sum(temp.velData.T_Skm1_Sk(:,1:3).^2,2));
scale(1:5) = scale(5);

parfor i = 1:4
    switch i
        case 1
            kittiCamData = genKittiCamB4(kittiPath, plotTforms, [], scale, 1);
            parsave('kittiCam1Data.mat', kittiCamData, 'cam1Data');
        case 2
            kittiCamData = genKittiCamB4(kittiPath, plotTforms, [], scale, 2);
            parsave('kittiCam2Data.mat', kittiCamData, 'cam2Data');
        case 3
            kittiCamData = genKittiCamB4(kittiPath, plotTforms, [], scale, 3);
            parsave('kittiCam3Data.mat', kittiCamData, 'cam3Data');
        case 4
            kittiCamData = genKittiCamB4(kittiPath, plotTforms, [], scale, 4);
            parsave('kittiCam4Data.mat', kittiCamData, 'cam4Data');
        otherwise
    end
end