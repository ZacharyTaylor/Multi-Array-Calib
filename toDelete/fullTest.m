%% user set variables

%data range
range = [65:100];

%set if sensor transforms have been precalculated
preprocessed = false;

%path to data
kittiPath = '/home/z/Documents/Datasets/Kitti/2011_09_30_drive_0028_sync';
%kittiPath =  'C:\Users\Zachary\Documents\Datasets\2011_09_26_drive_0035_sync';

%Sets if the sensor transforms will be plotted
plotTforms = true;

samples = 1000;

baseSensor = 1;

tformIdx = 1;

imageSensors = [2];

tform = cell(1,1);
tformVar = cell(1,1);

%% setup folders

%velodyne processing
addpath('./genKittiVel/');
addpath('./genKittiVel/libicp/matlab/');

%nav processing
addpath('./genKittiNav');

%camera processing
addpath('./genKittiCam/');

%hand eye calibration
addpath('./handEye/');
addpath('./handEye/errProp');

%calibration refinement
addpath('./imageMetric');

%optimization
addpath('./psopt');

%% process velodyne
% if(preprocessed)
%     load('kittiVelData.mat');
%     sensorData{tformIdx} = velData;
%     tform{tformIdx} = velData.T_Skm1_Sk(range,:);
%     tformVar{tformIdx} = velData.T_Cov_Skm1_Sk(range,:);
% else
%     velData = genKittiVel(kittiPath, plotTforms, range);
%     save('kittiVelData.mat', 'velData');
%     sensorData{tformIdx} = velData;
%     tform{tformIdx} = velData.T_Skm1_Sk;
%     tformVar{tformIdx} = velData.T_Cov_Skm1_Sk;
% end
% 
% tformIdx = tformIdx + 1;

%% process nav
% if(preprocessed)
%     load('kittiNavData.mat');
%     sensorData{tformIdx} = navData;
%     tform{tformIdx} = navData.T_Skm1_Sk(range,:);
%     tformVar{tformIdx} = navData.T_Cov_Skm1_Sk(range,:);
% else
%     navData = genKittiNav(kittiPath, plotTforms, range);
%     save('kittiNavData.mat', 'navData');
%     sensorData{tformIdx} = navData;
%     tform{tformIdx} = navData.T_Skm1_Sk;
%     tformVar{tformIdx} = navData.T_Cov_Skm1_Sk;
% end
% 
% tformIdx = tformIdx + 1;

%% process cameras
if(preprocessed)
    load('kittiCamDataFast.mat');
    sensorData{tformIdx} = camData{1};
    tform{tformIdx} = camData{1}.T_Skm1_Sk(range,:);
    tformVar{tformIdx} = camData{1}.T_Cov_Skm1_Sk(range,:);
    tformIdx = tformIdx + 1;
    
    sensorData{tformIdx} = camData{2};
    tform{tformIdx} = camData{2}.T_Skm1_Sk(range,:);
    tformVar{tformIdx} = camData{2}.T_Cov_Skm1_Sk(range,:);
    tformIdx = tformIdx + 1;
%      
%     sensorData{tformIdx} = camData{3};
%     tform{tformIdx} = camData{3}.T_Skm1_Sk(range,:);
%     tformVar{tformIdx} = camData{3}.T_Cov_Skm1_Sk(range,:);
%     tformIdx = tformIdx + 1;
%     
%     sensorData{tformIdx} = camData{4};
%     tform{tformIdx} = camData{4}.T_Skm1_Sk(range,:);
%     tformVar{tformIdx} = camData{4}.T_Cov_Skm1_Sk(range,:);
%     tformIdx = tformIdx + 1;
    
else
    scale = sqrt(sum(navData.T_Skm1_Sk(:,1:3).^2,2));
    camData = genKittiCamSBAWind(kittiPath, plotTforms, range, scale);
    save('kittiCamData.mat', 'camData');
    
    sensorData{tformIdx} = camData{1};
    tform{tformIdx} = camData{1}.T_Skm1_Sk(range,:);
    tformVar{tformIdx} = camData{1}.T_Cov_Skm1_Sk(:,:,range);
    tformIdx = tformIdx + 1;
    
    sensorData{tformIdx} = camData{2};
    tform{tformIdx} = camData{2}.T_Skm1_Sk(range,:);
    tformVar{tformIdx} = camData{2}.T_Cov_Skm1_Sk(:,:,range);
    tformIdx = tformIdx + 1;
    
    sensorData{tformIdx} = camData{3};
    tform{tformIdx} = camData{3}.T_Skm1_Sk(range,:);
    tformVar{tformIdx} = camData{3}.T_Cov_Skm1_Sk(:,:,range);
    tformIdx = tformIdx + 1;
    
    sensorData{tformIdx} = camData{4};
    tform{tformIdx} = camData{4}.T_Skm1_Sk(range,:);
    tformVar{tformIdx} = camData{4}.T_Cov_Skm1_Sk(:,:,range);
    tformIdx = tformIdx + 1;
end

numSensors = tformIdx - 1;

%% find rotations
tform = tform';
tformVar = tformVar';

options = optimset('MaxFunEvals',100000,'MaxIter',5000);

estVec = zeros(numSensors-1,3);
keep = true(size(tform{1},1),1);
for i = 1:numSensors-1
    temp = Kabsch(tform{1}(:,4:6)',tform{i+1}(:,4:6)',1./sqrt((max(tformVar{1}(:,4:6)') + max(tformVar{i+1}(:,4:6)'))));
    estVec(i,:) = rot2vec(temp);
    k = rejectProb( {tform{1}(keep,:);tform{i+1}(keep,:)}, {tformVar{1}(keep,:);tformVar{i+1}(keep,:)}, estVec(i,:) );
    keep(keep) = k;
end

for i = 1:size(tform,1)
    tform{i} = tform{i}(keep,:);
    tformVar{i} = tformVar{i}(keep,:);
end

%get tmat
tformMat = cell(size(tform));
for i = 1:size(tform,1)
    tformMat{i} = zeros(size(tform{i},1),12);
    for j = 1:size(tform{i},1)
        temp = vec2tran(tform{i}(j,:)');
        r = temp(1:3,1:3);
        t = temp(1:3,4);
        tformMat{i}(j,:) = [r(:)' t(:)'];
    end
end

RVec = fminsearch(@(estVec) systemProb( tform, tformVar, estVec ),estVec, options);

%convert rot vector to rotmats
RMat = zeros(3,3,size(RVec,1)+1);
for j = 2:size(RMat,3)
    RMat(:,:,j) = vec2rot(RVec(j-1,:)');
end
RMat(:,:,1) = eye(3);

estVecT = zeros(numSensors-1,3);
keep = true(size(tform{1},1),1);
for i = 1:numSensors-1
    V = 1./sqrt((max(tformVar{1}(:,1:3)') + max(tformVar{i+1}(:,1:3)')));

%     Ri = zeros(3*size(tform{1},1),3+size(tform{1},1));
%     for j = 1:size(tform{1},1)
%         Ri(3*j-2:3*j,1:3) = vec2rot(tform{i+1}(j,4:6)') - eye(3);
%         Ri(3*j-2:3*j,j+3) = tform{i+1}(j,1:3)';
%         Ri(3*j-2:3*j,:) = V(j).* Ri(3*j-2:3*j,:);
%     end
%     t1 = zeros(3*size(tform{1},1),1);
%     for j = 1:size(tform{1},1)
%         t1(3*j-2:3*j) = RMat(:,:,i+1)*tform{1}(j,1:3)';
%         t1(3*j-2:3*j) = V(j).* t1(3*j-2:3*j);
%     end

    Ri = zeros(3*size(tform{1},1),3);
    for j = 1:size(tform{1},1)
        Ri(3*j-2:3*j,1:3) = vec2rot(tform{i+1}(j,4:6)') - eye(3);
        Ri(3*j-2:3*j,:) = V(j).* Ri(3*j-2:3*j,:);
    end
    t1 = zeros(3*size(tform{1},1),1);
    for j = 1:size(tform{1},1)
        t1(3*j-2:3*j) = RMat(:,:,i+1)*tform{1}(j,1:3)' - tform{i+1}(j,1:3)';
        t1(3*j-2:3*j) = V(j).* t1(3*j-2:3*j);
    end
    
    temp = (Ri\t1)';
    estVecT(i,:) = temp(1:3);


%     for j = 1:size(tform{1},1)
%         tformMat{i+1}(j,10:12) = temp(j+3)*tformMat{i+1}(j,10:12);
%         tformVar{i+1}(j,1:3) = temp(j+3)*tformVar{i+1}(j,1:3);
%     end
 
    k = rejectProbT( {tformMat{1}(keep,:);tformMat{i+1}(keep,:)}, {tformVar{1}(keep,:);tformVar{i+1}(keep,:)}, estVecT(i,:), RMat(:,:,[1,i+1]) );
    keep(keep) = k;
end

for i = 1:size(tform,1)
    tform{i} = tform{i}(keep,:);
    tformMat{i} = tformMat{i}(keep,:);
    tformVar{i} = tformVar{i}(keep,:);
end

TVec = fminsearch(@(estVec) systemProbT( tformMat, tformVar, estVec, RMat),estVecT, options);
        
%bootstrap
tB = tform;
vB = tformVar;
TB = tformMat;
RVar = zeros(numSensors-1,3,50);
TVar = zeros(numSensors-1,3,50);

for i = 1:50
    bootIdx = datasample((1:size(tform{1},1))',size(tform{1},1));
    for j = 1:size(tform,1)
        tB{j} = tform{j}(bootIdx,:);
        vB{j} = tformVar{j}(bootIdx,:);
        TB{j} = tformMat{j}(bootIdx,:);
    end

    RVar(:,:,i) = fminsearch(@(estVec) systemProb( tB, vB, estVec ),estVec, options);

    %convert rot vector to rotmats
    RMat = zeros(3,3,size(RVar,1)+1);
    for j = 2:size(RMat,3)
        RMat(:,:,j) = vec2rot(RVar(j-1,:,i)');
    end
    RMat(:,:,1) = eye(3);

    TVar(:,:,i) = fminsearch(@(estVec) systemProbT( TB, vB, estVec, RMat),estVecT, options);
end

RVar = var(RVar(1,:,:),0,3);
TVar = var(TVar(1,:,:),0,3);

TVec = [0,0,0;TVec];
RVec = [0,0,0;RVec];
TVar = [0,0,0;TVar];
RVar = [0,0,0;RVar];

TOut = OptValsLev(TVec,TVar, RVec, RVar, 1, sensorData, range, imageSensors);
