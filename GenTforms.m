%% user set variables

%path to data
bagFile = 'handheld2.bag';

%topics
viconTopics = {'/auk/vrpn_client/raw_transform'};
viTopics = {'/auk/rovio/odometry'};
camTopics = {'/auk/cam0/image_raw','/auk/cam1/image_raw'};

%calibration information
camCalib = cell(size(camTopics));
camCalib{1}.D = [-4.478142052874399e-04;0.010562397976770;-0.008624781810349;0.001417044096805];
camCalib{1}.K = [4.687780324475454e+02,0,3.695881820819744e+02;0,4.684557680598397e+02,2.270404361213856e+02;0,0,1];
camCalib{1}.DistModel = 'equi';

camCalib{2}.D = [-0.001385876978281;0.012381761163228;-0.013693905520275;0.007085914973108];
camCalib{2}.K = [4.714127555100894e+02,0,3.918589543812900e+02;0,4.710971643451415e+02,2.356112806653724e+02;0,0,1];
camCalib{2}.DistModel = 'equi';

%Sets if the sensor transforms will be plotted
plotTforms = false;

%% setup folders
CalibPath(true);

%% process sensors

bag = rosbag(bagFile);

for i = 1:length(viconTopics)
    viconData = GenVicon(bag, plotTforms, viconTopics{i}, []);
    ParSave(['./storedTforms/' 'Vicon' num2str(i) 'Data.mat'], viconData, ['Vicon' num2str(i) 'Data']);
end

for i = 1:length(viTopics)
    viData = GenVIO(bag, plotTforms, viTopics{i}, []);
    ParSave(['./storedTforms/' 'VI' num2str(i) 'Data.mat'], viData, ['VI' num2str(i) 'Data']);
end

for i = 1:length(camTopics)
    camData = GenCam(bag, plotTforms, camTopics{i}, camCalib{i}, []);
    ParSave(['./storedTforms/' 'Cam' num2str(i) 'Data.mat'], camData, ['Cam' num2str(i) 'Data']);
end
    
CalibPath(false);