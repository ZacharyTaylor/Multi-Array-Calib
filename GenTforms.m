% this script generates all the required transforms for the kitti and
% shrimp dataset (currently wont do the camera as I am tweaking it)

%% user set variables

%path to data
%dataPath = 'C:\Users\Zachary\Documents\Datasets\IJRR-Dataset-1\'; dataset = 'Ford';
dataPath = 'C:\Users\Zachary\Documents\Datasets\Shrimp\high-clutter-2\'; dataset = 'Shrimp';
%dataPath = 'C:\Users\Zachary\Documents\Datasets\Kitti\2011_10_03_drive_0027_extract\'; dataset = 'Kitti';
%dataPath = 'C:\Users\Zachary\Documents\Datasets\Kitti\2011_09_26_drive_0035_extract\'; dataset = 'Kitti';

%Sets if the sensor transforms will be plotted
plotTforms = false;

%% setup folders
CalibPath(true);

%% process sensors

%do things in parrallel to save time
for i = 1
    switch i
        case 1
            VelData = GenVel(dataPath, plotTforms, [], dataset);
            ParSave(['./storedTforms/' dataset 'VelData3.mat'], VelData, 'velData');
        case 2
            NavData = GenNav(dataPath, plotTforms, [], dataset);
            ParSave(['./storedTforms/'  dataset 'NavData.mat'], NavData, 'navData'); 
        case 3
            CamData = GenCam(dataPath, plotTforms, [], dataset, 1);
            ParSave(['./storedTforms/' dataset 'Cam1Data.mat'], CamData, 'cam1Data');
        case 4
            CamData = GenCam(dataPath, plotTforms, [], dataset, 2);
            ParSave(['./storedTforms/' dataset 'Cam2Data.mat'], CamData, 'cam2Data');
        case 5
            CamData = GenCam(dataPath, plotTforms, [], dataset, 3);
            ParSave(['./storedTforms/' dataset 'Cam3Data.mat'], CamData, 'cam3Data');
        case 6
            CamData = GenCam(dataPath, plotTforms, [], dataset, 4);
            ParSave(['./storedTforms/' dataset 'Cam4Data.mat'], CamData, 'cam4Data');
        case 7
            %Kitti only has 4 cameras
            if(~strcmpi(dataset,'Kitti'))
                CamData = GenCam(dataPath, plotTforms, [], dataset, 5);
                ParSave(['./storedTforms/' dataset 'Cam5Data.mat'], CamData, 'cam5Data');
            end
        case 8
            %only shrimp has 6 cameras
            if(strcmpi(dataset,'Shrimp'))
                CamData = GenCam(dataPath, plotTforms, [], dataset, 6);
                ParSave(['./storedTforms/' dataset 'Cam6Data.mat'], CamData, 'cam6Data');
            end
        otherwise
            errror('Parfor setup incorrectly');
    end
end
    
CalibPath(false);
%delete(gcp);