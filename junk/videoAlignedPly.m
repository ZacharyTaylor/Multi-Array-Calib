load('Test_4_Res.mat');
pos = zeros(6,6,25);
for i = 1:25
    pos(:,:,i) = outT{i,1};
end

pos = median(pos,3);

range = 50:3049;

%% setup folders

%contains most of the presentable code
addpath('./finalClean');

addpath('./tformInterp');
addpath('./imageMetric');

addpath('./genKittiCam');

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
load('kittiCam1Data.mat');
sensorData{tformIdx,1} = cam1Data;
tformIdx = tformIdx + 1;

load('kittiCam2Data.mat');
sensorData{tformIdx,1} = cam2Data;
tformIdx = tformIdx + 1;

load('kittiCam3Data.mat');
sensorData{tformIdx,1} = cam3Data;
tformIdx = tformIdx + 1;

load('kittiCam4Data.mat');
sensorData{tformIdx,1} = cam4Data;
tformIdx = tformIdx + 1;

%% find transformations

for i = 1:length(sensorData)
    if(i > 1)
        sensorData{i} = matchTforms(sensorData{i}, sensorData{1},range, false);
    else
        sensorData{i}.T_Skm1_Sk = sensorData{i}.T_Skm1_Sk(range,:);
        sensorData{i}.T_S1_Sk = sensorData{i}.T_S1_Sk(range,:);
        sensorData{i}.T_Cov_Skm1_Sk = sensorData{i}.T_Cov_Skm1_Sk(range,:);
        sensorData{i}.time = sensorData{i}.time(range,:);
        sensorData{i}.files = sensorData{i}.files(range,:);
    end
end

T = eye(4);
cmap = colormap(jet(256));
for frame = 1:size(sensorData{1}.files,1)
    
    frame
    
    %postion
    velT1 = dlmread([ sensorData{1}.folder  sensorData{1}.files(frame).name],' ');
    
    T = T/(vec2tran(sensorData{1,1}.T_Skm1_Sk(frame,:)'));
    velAbs = (T*[velT1(:,1:3),ones(size(velT1,1),1)]')';
    
    %intensity
    velInt = cmap(round(255*velT1(:,4)+1),:);
    velT1 = velT1(:,1:3);
       
    lidar = gpuArray(single(velT1));
    
    cam0 = colourPly( sensorData{2}, lidar, pos(3,:), frame);
    cam1 = colourPly( sensorData{3}, lidar, pos(4,:), frame);
    cam2 = colourPly( sensorData{4}, lidar, pos(5,:), frame);
    cam3 = colourPly( sensorData{5}, lidar, pos(6,:), frame);

    
    DATA = struct;
    DATA.vertex.x = velT1(:,1);
    DATA.vertex.y = velT1(:,2);
    DATA.vertex.z = velT1(:,3);
    
    DATA.vertex.XABS = velAbs(:,1);
    DATA.vertex.YABS = velAbs(:,2);
    DATA.vertex.ZABS = velAbs(:,3);
    
    DATA.vertex.INTR = velInt(:,1);
    DATA.vertex.INTG = velInt(:,2);
    DATA.vertex.INTB = velInt(:,3);
    
    DATA.vertex.C0R = cam0(:,1);
    DATA.vertex.C0G = cam0(:,2);
    DATA.vertex.C0B = cam0(:,3);
    
    DATA.vertex.C1R = cam1(:,1);
    DATA.vertex.C1G = cam1(:,2);
    DATA.vertex.C1B = cam1(:,3);
    
    DATA.vertex.C2R = cam2(:,1);
    DATA.vertex.C2G = cam2(:,2);
    DATA.vertex.C2B = cam2(:,3);
    
    DATA.vertex.C3R = cam3(:,1);
    DATA.vertex.C3G = cam3(:,2);
    DATA.vertex.C3B = cam3(:,3);
    
    out = ['/home/z/Documents/Houdini/ICRA2015/lidar2/', num2str(frame,'%04i'),'.ply'];
    
    ply_write(DATA,out);
end
