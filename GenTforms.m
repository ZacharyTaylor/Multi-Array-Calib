% this script generates all the required transforms for the kitti and
% shrimp dataset (currently wont do the camera as I am tweaking it)

%% user set variables

%dataset type
dataset = 'Kitti';

%path to data
dataPath = '/home/z/Documents/Datasets/Kitti/2011_10_03_drive_0027_extract/';

%Sets if the sensor transforms will be plotted
plotTforms = false;

%% setup folders

%velodyne processing
addpath('./genVel/');
addpath('./genVel/velicp/matlab/');

%nav processing
addpath('./genNav');

%cam processing
addpath('./genCam');

addpath('./handEye');

%% process sensors

%do things in parrallel to save time
for i = 2
    switch i
        case 1
            kittiVelData = GenVel(dataPath, plotTforms, [], dataset);
            parsave('kittiVelData.mat', kittiVelData, 'velData');
        case 2
            NavData = GenNav(dataPath, plotTforms, [], dataset);
            parsave([dataset 'NavData.mat'], NavData, 'navData'); 
        case 3
            CamData = GenCam(dataPath, plotTforms, [], dataset, 1);
            parsave([dataset 'Cam1Data.mat'], CamData, 'cam1Data');
        case 4
            CamData = GenCam(dataPath, plotTforms, [], dataset, 2);
            parsave([dataset 'Cam2Data.mat'], CamData, 'cam2Data');
        case 5
            CamData = GenCam(dataPath, plotTforms, [], dataset, 3);
            parsave([dataset 'Cam3Data.mat'], CamData, 'cam3Data');
        case 6
            CamData = GenCam(dataPath, plotTforms, [], dataset, 4);
            parsave([dataset 'Cam4Data.mat'], CamData, 'cam4Data');
        case 7
            %Kitti only has 4 cameras
            if(~strcmpi(dataset,'Kitti'))
                CamData = GenCam(dataPath, plotTforms, [], dataset, 5);
                parsave([dataset 'Cam5Data.mat'], CamData, 'cam5Data');
            end
        case 8
            %only shrimp has 6 cameras
            if(strcmpi(dataset,'Shrimp'))
                CamData = GenCam(dataPath, plotTforms, [], dataset, 6);
                parsave([dataset 'Cam6Data.mat'], CamData, 'cam6Data');
            end
        otherwise
            errror('Parfor setup incorrectly');
    end
end
    
delete(gcp);