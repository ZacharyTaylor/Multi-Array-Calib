%% user set variables

%path to data
bagFile = 'handheld2.bag';

%topics
viconTopics = {'/auk/vrpn_client/raw_transform'};
camTopics = {'/auk/cam0/image_raw','/auk/cam1/image_raw'};
camCalibTopics = {'/auk/cam0/calibration','/auk/cam1/calibration'};

%Sets if the sensor transforms will be plotted
plotTforms = false;

%% setup folders
CalibPath(true);

%% process sensors

%do things in parrallel to save time
for i = 1:(length(viconTopics)+length(camTopics))
    if(i <= length(camTopics))
        camData = GenCam(bag, plotTforms, camTopics{i}, camCalibTopics{i}, []);
        ParSave(['./storedTforms/' 'Cam' num2str(i) 'Data.mat'], camData, ['Cam' num2str(i) 'Data']);
    elseif(i <= (length(viconTopics)+length(camTopics)))
        viconData = GenVicon(bag, plotTforms, viconTopics{i-length(camTopics)}, []);
        ParSave(['./storedTforms/' 'Vicon' num2str(i-length(camTopics)) 'Data.mat'], viconData, ['Vicon' num2str(i-length(camTopics)) 'Data']);
    end
end
    
CalibPath(false);