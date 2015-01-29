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
    
sensorData{tformIdx} = camData{2};
tform{tformIdx} = camData{2}.T_Skm1_Sk(range,:);
tformVar{tformIdx} = camData{2}.T_Cov_Skm1_Sk(range,:);
tformIdx = tformIdx + 1;
% 
% sensorData{tformIdx} = camData{3};
% tform{tformIdx} = camData{3}.T_Skm1_Sk(range,:);
% tformVar{tformIdx} = camData{3}.T_Cov_Skm1_Sk(range,:);
% tformIdx = tformIdx + 1;
% 
% sensorData{tformIdx} = camData{4};
% tform{tformIdx} = camData{4}.T_Skm1_Sk(range,:);
% tformVar{tformIdx} = camData{4}.T_Cov_Skm1_Sk(range,:);
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

RErr = zeros(1,6,10);
TErr = zeros(1,6,10);

RErrEqual = zeros(1,6,10);
TErrEqual = zeros(1,6,10);

RVar = zeros(1,6,10);
TVar = zeros(1,6,10);

options = optimset('MaxFunEvals',100000,'MaxIter',5000);

for w = 1:100
        w
    for l = 3000%l = 100:100:1000

        %get random index to use for scans
        idx = randi(4900-l+10)+5;
        idx = (idx:idx+l)';

        %get corrosponding transformations
        for i = 1:size(tform,1)
            tform{i} = tformBase{i}(idx,:);
            tformVar{i} = tformVarBase{i}(idx,:);
        end

        %perform pair based simple matching for comparison
        temp = Kabsch(tform{1}(:,4:6)',tform{2}(:,4:6)');
        RErrEqual(w,1:3,l/100) = rot2vec(temp)';
        Ri = zeros(3*size(tform{1},1),3);
        for j = 1:size(tform{1},1)
            Ri(3*j-2:3*j,1:3) = vec2rot(tform{2}(j,4:6)') - eye(3);
        end
        t1 = zeros(3*size(tform{1},1),1);
        for j = 1:size(tform{1},1)
            t1(3*j-2:3*j) = temp*tform{1}(j,1:3)' - tform{2}(j,1:3)';
        end
        temp = (Ri\t1)';
        TErrEqual(w,1:3,l/100) = temp(1:3);
        
        temp = Kabsch(tform{1}(:,4:6)',tform{3}(:,4:6)');
        RErrEqual(w,4:6,l/100) = rot2vec(temp)';
        Ri = zeros(3*size(tform{1},1),3);
        for j = 1:size(tform{1},1)
            Ri(3*j-2:3*j,1:3) = vec2rot(tform{3}(j,4:6)') - eye(3);
        end
        t1 = zeros(3*size(tform{1},1),1);
        for j = 1:size(tform{1},1)
            t1(3*j-2:3*j) = temp*tform{1}(j,1:3)' - tform{2}(j,1:3)';
        end
        temp = (Ri\t1)';
        TErrEqual(w,4:6,l/100) = temp(1:3);
            
        
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
        RErr(w,:,l/100) = [temp(1,:), temp(2,:)];
        
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
            estVecT(i,:) = temp(1:3);
            
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
        TErr(w,:,l/100) = [temp(1,:), temp(2,:)];

        
        %bootstrap
        tB = tform;
        vB = tformVar;
        TB = tformMat;
        RVec = zeros(numSensors-1,3,50);
        TVec = zeros(numSensors-1,3,50);

        for i = 1:50
            bootIdx = datasample((1:size(tform{1},1))',size(tform{1},1));
            for j = 1:size(tform,1)
                tB{j} = tform{j}(bootIdx,:);
                vB{j} = tformVar{j}(bootIdx,:);
                TB{j} = tformMat{j}(bootIdx,:);
            end

            RVec(:,:,i) = fminsearch(@(estVec) systemProb( tB, vB, estVec ),estVec, options);

            %convert rot vector to rotmats
            RMat = zeros(3,3,size(RVec,1)+1);
            for j = 2:size(RMat,3)
                RMat(:,:,j) = vec2rot(RVec(j-1,:,i)');
            end
            RMat(:,:,1) = eye(3);

            TVec(:,:,i) = fminsearch(@(estVec) systemProbT( TB, vB, estVec, RMat),estVecT, options);
        end

        %find variance
        RVar(w,:,l/100) = [var(RVec(1,:,:),0,3),var(RVec(2,:,:),0,3)];
        TVar(w,:,l/100) = [var(TVec(1,:,:),0,3),var(TVec(2,:,:),0,3)];
        
        save('RErr2.mat', 'RErr');
        save('TErr2.mat', 'TErr');
        
        save('RErrEqual2.mat', 'RErrEqual');
        save('TErrEqual2.mat', 'TErrEqual');
        
        save('RVar2.mat', 'RVar');
        save('TVar2.mat', 'TVar');
    end
end
     
%% plot results

%ensure latest results loaded
load('RErr.mat');
RErr = RErr(1:50,:,:);
load('RVar.mat');
RVar = RVar(1:50,:,:);
load('TErr.mat');
TErr = TErr(1:50,:,:);
load('TVar.mat');
TVar = TVar(1:50,:,:);
load('RErrEqual.mat');
RErrEqual = RErrEqual(1:50,:,:);
load('TErrEqual.mat');
TErrEqual = TErrEqual(1:50,:,:);
%convert from position to error
actualR = [1.20134997604243,-1.19307958456946,1.21012672103152,-0.0148243143349357,0.00203019194952995,0.000770383764607292];
actualT = [-0.00406976600000000,-0.0763161800000000,-0.271780600000000,0.808675900000000,-0.319555900000000,0.799723100000000];

RErr = RErr - repmat(actualR,[size(RErr,1),1,size(RErr,3)]);
RErrEqual = RErrEqual - repmat(actualR,[size(RErrEqual,1),1,size(RErrEqual,3)]);
TErr = TErr - repmat(actualT,[size(TErr,1),1,size(TErr,3)]);
TErrEqual = TErrEqual - repmat(actualT,[size(TErrEqual,1),1,size(TErrEqual,3)]);

%get means
RErr = mean(abs(RErr),1);
RErr = reshape(RErr,6,100)';
TErr = mean(abs(TErr),1);
TErr = reshape(TErr,6,100)';

RErrEqual = mean(abs(RErrEqual),1);
RErrEqual = reshape(RErrEqual,6,100)';
TErrEqual = mean(abs(TErrEqual),1);
TErrEqual = reshape(TErrEqual,6,100)';

RVar = mean(RVar,1);
RVar = reshape(RVar,6,100)';
TVar = mean(TVar,1);
TVar = reshape(TVar,6,100)';

%convert to angular error in degrees
for i = 1:100
    pop = mvnrnd(RErr(i,:),diag(RVar(i,:)),100);
    temp = zeros(100,2);
    %use sampling approach to transfer variance
    for j = 1:100
        q1 = dcm2quat(vec2rot(pop(j,1:3)'));
        q2 = dcm2quat(vec2rot(pop(j,4:6)'));
        temp(j,:) = [abs(acosd(q1(1))),abs(acosd(q2(1)))];
    end
    RVar(i,1:2) = std(temp);
    
    q1 = dcm2quat(vec2rot(RErr(i,1:3)'));
    q2 = dcm2quat(vec2rot(RErr(i,4:6)'));
    RErr(i,1:2) = [abs(acosd(q1(1))),abs(acosd(q2(1)))];
    
    q1 = dcm2quat(vec2rot(RErrEqual(i,1:3)'));
    q2 = dcm2quat(vec2rot(RErrEqual(i,4:6)'));
    RErrEqual(i,1:2) = [abs(acosd(q1(1))),abs(acosd(q2(1)))];
end
RVar = RVar(:,1:2);
RErr = RErr(:,1:2);
RErrEqual = RErrEqual(:,1:2);

%convert to translational error
for i = 1:100
    pop = mvnrnd(TErr(i,:),diag(TVar(i,:)),100);
    temp = zeros(100,2);
    %use sampling approach to transfer variance
    for j = 1:100
        q1 = sqrt(sum(pop(j,1:3).^2));
        q2 = sqrt(sum(pop(j,4:6).^2));
        temp(j,:) = [abs(q1(1)),abs(q2(1))];
    end
    TVar(i,1:2) = std(temp);
    
    q1 = sqrt(sum(TErr(i,1:3).^2));
    q2 = sqrt(sum(TErr(i,4:6).^2));
    TErr(i,1:2) = [abs(q1),abs(q2)];
    
    q1 = sqrt(sum(TErrEqual(i,1:3).^2));
    q2 = sqrt(sum(TErrEqual(i,4:6).^2));
    TErrEqual(i,1:2) = [abs(q1),abs(q2)];
end
TVar = TVar(:,1:2);
TErr = TErr(:,1:2);
TErrEqual = TErrEqual(:,1:2);

x = (10:10:1000)';

addpath('./plotBounds');

%plot
close all
figure
hold on;

%boundedline(x,RErr(:,1),RVar(:,1),'-b');
%boundedline(x,RErr(:,2),RVar(:,2),'-r');
plot(x,RErr(:,1),'-b');
plot(x,RErr(:,2),'-r');
plot(x,RErrEqual(:,1),'-g');
plot(x,RErrEqual(:,2),'-k');
title('Rotation Error (Degrees)');
axis([10 1000 0 10]);
set(gca,'layer','top');
set(gcf,'color','w');

figure
hold on;

%boundedline(x,TErr(:,1),TVar(:,1),'-b');
%boundedline(x,TErr(:,2),TVar(:,2),'-r');
plot(x,TErr(:,1),'-b');
plot(x,TErr(:,2),'-r');
plot(x,TErrEqual(:,1),'-g');
plot(x,TErrEqual(:,2),'-k');
axis([10 1000 0 20]);
set(gca,'layer','top');
title('Translation Error (m)');
set(gcf,'color','w');



