%% user set variables

%path to data
bagFile = 'handheld2.bag';

%topics
viconTopics = {'/auk/vrpn_client/raw_transform'};
viTopics = {'/auk/rovio/odometry'};
camTopics = {'/auk/cam0/image_raw','/auk/cam1/image_raw'};
camCalibTopics = {'/auk/cam0/calibration','/auk/cam1/calibration'};

%Sets if the sensor transforms will be plotted
plotTforms = false;

%% setup folders
CalibPath(true);

%% process sensors

bag = rosbag(bagFile);

% for i = 1:length(camTopics)
%     camData = GenCam(bag, plotTforms, camTopics{i}, camCalibTopics{i}, []);
%     ParSave(['./storedTforms/' 'Cam' num2str(i) 'Data.mat'], camData, ['Cam' num2str(i) 'Data']);
% end

% for i = 1:length(viconTopics)
%     viconData = GenVicon(bag, plotTforms, viconTopics{i}, []);
%     ParSave(['./storedTforms/' 'Vicon' num2str(i) 'Data.mat'], viconData, ['Vicon' num2str(i) 'Data']);
% end

for i = 1:length(viTopics)
    viData = GenVIO(bag, plotTforms, viTopics{i}, []);
    ParSave(['./storedTforms/' 'VI' num2str(i) 'Data.mat'], viData, ['VI' num2str(i) 'Data']);
end
    
CalibPath(false);