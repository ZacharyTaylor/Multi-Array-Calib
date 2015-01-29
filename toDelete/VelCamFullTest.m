% this script generates the first set of results in the paper
% It calculates the transformation between the gps and velodyne for drive
% 28 of the kitti data set using the presented method and a simple equal
% weighted least squares method

%% user set variables

%data range
range = [2:5000];

%path to data
kittiPath = '/home/z/Documents/Datasets/Kitti/2011_09_30_drive_0028_sync';

%Sets if the sensor transforms will be plotted
plotTforms = false;

samples = 1000;

baseSensor = 1;
imageSensors = 2;

tformIdx = 1;

tform = cell(1,1);
tformVar = cell(1,1);

%% setup folders

%velodyne processing
addpath('./genKittiVel/');
addpath('./genKittiVel/libicp/matlab/');

%nav processing
addpath('./genKittiNav');

%hand eye calibration
addpath('./handEye/');
addpath('./handEye/errProp');

addpath('./imageMetric');

%% process velodyne
load('kittiVelData.mat');
sensorData{tformIdx} = velData;
tform{tformIdx} = velData.T_Skm1_Sk(range,:);
tformVar{tformIdx} = velData.T_Cov_Skm1_Sk(range,:);
tformIdx = tformIdx + 1;

%% process cam
load('kittiCamData.mat');
sensorData{tformIdx} = camData{1};
tform{tformIdx} = camData{1}.T_Skm1_Sk(range,:);
tformVar{tformIdx} = camData{1}.T_Cov_Skm1_Sk(range,:);
tformIdx = tformIdx + 1;
    
% sensorData{tformIdx} = camData{2};
% tform{tformIdx} = camData{1}.T_Skm1_Sk(range,:);
% tformVar{tformIdx} = camData{1}.T_Cov_Skm1_Sk(range,:);
% tformIdx = tformIdx + 1;
% 
% sensorData{tformIdx} = camData{3};
% tform{tformIdx} = camData{1}.T_Skm1_Sk(range,:);
% tformVar{tformIdx} = camData{1}.T_Cov_Skm1_Sk(range,:);
% tformIdx = tformIdx + 1;
% 
% sensorData{tformIdx} = camData{4};
% tform{tformIdx} = camData{1}.T_Skm1_Sk(range,:);
% tformVar{tformIdx} = camData{1}.T_Cov_Skm1_Sk(range,:);
% tformIdx = tformIdx + 1;

%% process nav
% load('kittiNavData.mat');
% sensorData{tformIdx} = navData;
% tform{tformIdx} = navData.T_Skm1_Sk(range,:);
% tformVar{tformIdx} = navData.T_Cov_Skm1_Sk(range,:);
% tformIdx = tformIdx + 1;
% 

numSensors = tformIdx - 1;

%% find transformations
tform = tform';
tformVar = tformVar';

tformBase = tform;
tformVarBase = tformVar;

RErr = zeros(10,3);
TErr = zeros(10,3);

RLevErr = zeros(10,3);
TLevErr = zeros(10,3);

RVar = zeros(10,3);
TVar = zeros(10,3);

RLevVar = zeros(10,3);
TLevVar = zeros(10,3);

options = optimset('MaxFunEvals',100000,'MaxIter',5000);

for w = 1:100
    w
    l = 500;

        %get random index to use for scans
        idx = randi(5000-l+10)+5;
        idx = (idx:idx+l)';

        %get corrosponding transformations
        for i = 1:size(tform,1)
            tform{i} = tformBase{i}(idx,:);
            tformVar{i} = tformVarBase{i}(idx,:);
        end
            
        %perform inital kabsch matching for rotation to first sensor and
        %reject large errors
        estVec = zeros(numSensors-1,3);
        keep = true(size(tform{1},1),1);
        for i = 1:numSensors-1
            weight = 1./sqrt((max(tformVar{1}(:,4:6)') + max(tformVar{i+1}(:,4:6)')));
            [temp,lrms] = Kabsch(tform{1}(:,4:6)',tform{i+1}(:,4:6)',weight);
            estVec(i,:) = rot2vec(temp);
            k = rejectProb( {tform{1}(keep,:);tform{i+1}(keep,:)}, {tformVar{1}(keep,:);tformVar{i+1}(keep,:)}, estVec(i,:) );
            keep(keep) = k;
        end

        %remove frames with large errors
        for i = 1:size(tform,1)
            tform{i} = tform{i}(keep,:);
            tformVar{i} = tformVar{i}(keep,:);
        end

        %get matrix form of rotation
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

        %refine rotation estimate and record result
        temp = fminsearch(@(estVec) systemProb( tform, tformVar, estVec ),estVec, options);
        RErr(w,:) = temp;
        
        %convert rot vector to rotmats
        RMat = zeros(3,3,size(temp,1)+1);
        for j = 2:size(RMat,3)
            RMat(:,:,j) = vec2rot(temp(j-1,:)');
        end
        RMat(:,:,1) = eye(3);
        
        %perform simple least squares solution for translation estimate
        estVecT = zeros(numSensors-1,3);
        keep = true(size(tform{1},1),1);
        for i = 1:numSensors-1
            V = 1./sqrt((max(tformVar{1}(:,1:3)') + max(tformVar{i+1}(:,1:3)')));
                
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
            estVecT(i,:) = temp;
            
            k = rejectProbT( {tformMat{1}(keep,:);tformMat{i+1}(keep,:)}, {tformVar{1}(keep,:);tformVar{i+1}(keep,:)}, estVecT(i,:), RMat(:,:,[1,i+1]) );
            keep(keep) = k;
        end
        
        %remove points with large errors
        for i = 1:size(tform,1)
            tform{i} = tform{i}(keep,:);
            tformMat{i} = tformMat{i}(keep,:);
            tformVar{i} = tformVar{i}(keep,:);
        end
        
        %refine estimate
        temp = fminsearch(@(estVec) systemProbT( tformMat, tformVar, estVec, RMat),estVecT, options);
        TErr(w,:) = temp(1,:);

        
        %bootstrap
        tB = tform;
        vB = tformVar;
        TB = tformMat;
        RVec = zeros(50,3);
        TVec = zeros(50,3);

        for i = 1:50
            bootIdx = datasample((1:size(tform{1},1))',size(tform{1},1));
            for j = 1:size(tform,1)
                tB{j} = tform{j}(bootIdx,:);
                vB{j} = tformVar{j}(bootIdx,:);
                TB{j} = tformMat{j}(bootIdx,:);
            end

            RVec(i,:) = fminsearch(@(estVec) systemProb( tB, vB, estVec ),estVec, options);

            %convert rot vector to rotmats
            RMat = zeros(3,3,size(RVec,1)+1);
            for j = 2:size(RMat,3)
                RMat(:,:,j) = vec2rot(RVec(i,:)');
            end
            RMat(:,:,1) = eye(3);

            TVec(i,:) = fminsearch(@(estVec) systemProbT( TB, vB, estVec, RMat),estVecT, options);
        end

        %find variance
        RVar(w,:) = var(RVec);
        TVar(w,:) = var(TVec);
        
        RM = [0,0,0;RErr(w,:)];
        RV = [0,0,0;RVar(w,:)];
        TM = [0,0,0;TErr(w,:)];
        TV = [0,0,0;TVar(w,:)];
        [T,TV] = OptValsGom(TM,TV,RM,RV, baseSensor,sensorData, range, imageSensors);
        
        RLevErr(w,:) = T(4:6);
        TLevErr(w,:) = T(1:3);
        RLevVar(w,:) = TV(4:6);
        TLevVar(w,:) = TV(1:3);

%         save('VelRErr.mat','RErr');
%         save('VelTErr.mat','TErr');
%         
%         save('VelRLevErr.mat','RLevErr');
%         save('VelTLevErr.mat','TLevErr');
% 
%         save('VelRVar.mat','RVar');
%         save('VelTVar.mat','TVar');
% 
%         save('VelRLevVar.mat','RLevVar');
%         save('VelTLevVar.mat','TLevVar');
end
     
%% plot results

%ensure latest results loaded
load('VelRErr.mat');
RErr = RErr(1:50,:,:);
load('VelRVar.mat');
RVar = RVar(1:50,:,:);
load('VelTErr.mat');
TErr = TErr(1:50,:,:);
load('VelTVar.mat');
TVar = TVar(1:50,:,:);
load('VelRLevErr.mat');
RLevErr = RLevErr(1:50,:,:);
load('VelTLevErr.mat');
TLevErr = TLevErr(1:50,:,:);

%convert from position to error
actualR = [1.20134997604243,-1.19307958456946,1.21012672103152];
actualT = [-0.00406976600000000,-0.0763161800000000,-0.271780600000000];

RErr = RErr - repmat(actualR,[size(RErr,1),1,size(RErr,3)]);
RLevErr = RLevErr - repmat(actualR,[size(RLevErr,1),1,size(RLevErr,3)]);
TErr = TErr - repmat(actualT,[size(TErr,1),1,size(TErr,3)]);
TLevErr = TLevErr - repmat(actualT,[size(TLevErr,1),1,size(TLevErr,3)]);

%get means
RErr = mean(abs(RErr),1);
RErr = reshape(RErr,6,100)';
TErr = mean(abs(TErr),1);
TErr = reshape(TErr,6,100)';

RLevErr = mean(abs(RLevErr),1);
RLevErr = reshape(RLevErr,6,100)';
TLevErr = mean(abs(TLevErr),1);
TLevErr = reshape(TLevErr,6,100)';

RVar = mean(RVar,1);
RVar = reshape(RVar,6,100)';
TVar = mean(TVar,1);
TVar = reshape(TVar,6,100)';

%convert to angular error in degrees
for i = 1:100
    pop = mvnrnd(RErr(i,:),diag(RVar(i,:)),100);
    temp = zeros(100,1);
    %use sampling approach to transfer variance
    for j = 1:100
        q1 = dcm2quat(vec2rot(pop(j,1:3)'));
        q2 = dcm2quat(vec2rot(pop(j,4:6)'));
        temp(j) = abs(acosd(q1(1)))+abs(acosd(q2(1)));
    end
    RVar(i,1) = std(temp,0,1);
    
    q1 = dcm2quat(vec2rot(RErr(i,1:3)'));
    q2 = dcm2quat(vec2rot(RErr(i,4:6)'));
    RErr(i,1) = abs(acosd(q1(1)))+abs(acosd(q2(1)));
    
    q1 = dcm2quat(vec2rot(RLevErr(i,1:3)'));
    q2 = dcm2quat(vec2rot(RLevErr(i,4:6)'));
    RLevErr(i,1) = abs(acosd(q1(1)))+abs(acosd(q2(1)));
end
RVar = RVar(:,1);
RErr = RErr(:,1);
RLevErr = RLevErr(:,1);

%convert to translational error
for i = 1:100
    pop = mvnrnd(TErr(i,:),diag(TVar(i,:)),100);
    temp = zeros(100,1);
    %use sampling approach to transfer variance
    for j = 1:100
        q1 = sqrt(sum(pop(j,1:3).^2));
        q2 = sqrt(sum(pop(j,4:6).^2));
        temp(j) = abs(q1(1))+abs(q2(1));
    end
    TVar(i,1) = std(temp,0,1);
    
    q1 = sqrt(sum(TErr(i,1:3).^2));
    q2 = sqrt(sum(TErr(i,4:6).^2));
    TErr(i,1) = abs(q1)+abs(q2);
    
    q1 = sqrt(sum(TLevErr(i,1:3).^2));
    q2 = sqrt(sum(TLevErr(i,4:6).^2));
    TLevErr(i,1) = abs(q1)+abs(q2);
end
TVar = TVar(:,1);
TErr = TErr(:,1);
TLevErr = TLevErr(:,1);

x = (10:10:1000)';

addpath('./plotBounds');

%plot
close all
figure
hold on;

boundedline(x,RErr(:,1),RVar(:,1),'-b');
plot(x,RLevErr(:,1),'-k');
title('Rotation Error (Degrees)');
axis([10 1000 0 5]);
set(gca,'layer','top');
set(gcf,'color','w');

figure
hold on;

boundedline(x,TErr(:,1),TVar(:,1),'-g');
plot(x,TLevErr(:,1),'-k');
axis([10 1000 0 5]);
set(gca,'layer','top');
title('Translation Error (m)');
set(gcf,'color','w');



