% this script generates the first set of results in the paper
% It calculates the transformation between the gps and velodyne for drive
% 28 of the kitti data set using the presented method and a simple equal
% weighted least squares method

%% user set variables

%data range
range = [2:5050];

%set if sensor transforms have been precalculated
preprocessed = true;

%path to data
kittiPath = '/home/z/Documents/Datasets/Kitti/2011_09_30_drive_0028_sync';

%Sets if the sensor transforms will be plotted
plotTforms = false;

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

%hand eye calibration
addpath('./handEye/');
addpath('./handEye/errProp');

%% process velodyne
if(preprocessed)
    load('kittiVelData.mat');
    sensorData{tformIdx} = velData;
    tform{tformIdx} = velData.T_Skm1_Sk(range,:);
    tformVar{tformIdx} = velData.T_Cov_Skm1_Sk(range,:);
else
    velData = genKittiVel(kittiPath, plotTforms, range);
    save('kittiVelData.mat', 'velData');
    sensorData{tformIdx} = velData;
    tform{tformIdx} = velData.T_Skm1_Sk;
    tformVar{tformIdx} = velData.T_Cov_Skm1_Sk;
end

tformIdx = tformIdx + 1;

%% process nav
if(preprocessed)
    load('kittiNavData.mat');
    sensorData{tformIdx} = navData;
    tform{tformIdx} = navData.T_Skm1_Sk(range,:);
    tformVar{tformIdx} = navData.T_Cov_Skm1_Sk(range,:);
else
    navData = genKittiNav(kittiPath, plotTforms, range);
    save('kittiNavData.mat', 'navData');
    sensorData{tformIdx} = navData;
    tform{tformIdx} = navData.T_Skm1_Sk;
    tformVar{tformIdx} = navData.T_Cov_Skm1_Sk;
end

tformIdx = tformIdx + 1;

numSensors = tformIdx - 1;

%% find transformations
tform = tform';
tformVar = tformVar';

tformBase = tform;
tformVarBase = tformVar;

RErr = zeros(100,3,100);
TErr = zeros(100,3,100);

RErrEqual = zeros(100,3,100);
TErrEqual = zeros(100,3,100);

RVar = zeros(100,3,100);
TVar = zeros(100,3,100);

options = optimset('MaxFunEvals',100000,'MaxIter',5000);

for w = 100%1:100
        w
    for l = 1000%10:10:1000
        l

        %get random index to use for scans
        idx = randi(5000-l+10)+5;
        idx = (idx:idx+l)';

        %get corrosponding transformations
        for i = 1:size(tform,1)
            tform{i} = tformBase{i}(idx,:);
            tformVar{i} = tformVarBase{i}(idx,:);
        end

        %perform pair based simple matching for comparison
        temp = Kabsch(tform{1}(:,4:6)',tform{2}(:,4:6)');
        RErrEqual(w,:,l/10) = rot2vec(temp)' - [-0.0148243143349357,0.00203019194952995,0.000770383764607292];
        Ri = zeros(3*size(tform{1},1),3);
        for j = 1:size(tform{1},1)
            Ri(3*j-2:3*j,1:3) = vec2rot(tform{2}(j,4:6)') - eye(3);
        end
        t1 = zeros(3*size(tform{1},1),1);
        for j = 1:size(tform{1},1)
            t1(3*j-2:3*j) = temp*tform{1}(j,1:3)' - tform{2}(j,1:3)';
        end
        temp = (Ri\t1)';
        TErrEqual(w,:,l/10) = temp(1:3) - [0.808675900000000,-0.319555900000000,0.799723100000000];
            
        
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
        RErr(w,:,l/10) = temp(1,:) - [-0.0148243143349357,0.00203019194952995,0.000770383764607292];
        
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
        TErr(w,:,l/10) = temp(1,:) - [0.808675900000000,-0.319555900000000,0.799723100000000];

        
        %bootstrap
        tB = tform;
        vB = tformVar;
        TB = tformMat;
        RVec = zeros(numSensors-1,3,100);
        TVec = zeros(numSensors-1,3,100);

        for i = 1:100
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
        RVar(w,:,l/10) = var(RVec(1,:,:),0,3);
        TVar(w,:,l/10) = var(TVec(1,:,:),0,3);
        
        save('RErr.mat', 'RErr');
        save('TErr.mat', 'TErr');
        
        save('RErrEqual.mat', 'RErrEqual');
        save('TErrEqual.mat', 'TErrEqual');
        
        save('RVar.mat', 'RVar');
        save('TVar.mat', 'TVar');
    end
end
     
%% plot results

%ensure latest results loaded
load('RErr.mat');
load('RVar.mat');
load('TErr.mat');
load('TVar.mat');
load('RErrEqual.mat');
load('TErrEqual.mat');

%get means
RErr = mean(abs(RErr),1);
RErr = reshape(RErr,3,100)';
TErr = mean(abs(TErr),1);
TErr = reshape(TErr,3,100)';

RErrEqual = mean(abs(RErrEqual),1);
RErrEqual = reshape(RErrEqual,3,100)';
TErrEqual = mean(abs(TErrEqual),1);
TErrEqual = reshape(TErrEqual,3,100)';

RVar = mean(RVar,1);
RVar = reshape(RVar,3,100)';
TVar = mean(sqrt(abs(TVar)),1);
TVar = reshape(TVar,3,100)';

%convert to roll, pitch, yaw in degrees
for i = 1:100
    pop = mvnrnd(RErr(i,:),diag(RVar(i,:)),100);
    temp = zeros(100,3);
    %use sampling approach to transfer variance
    for j = 1:100
        [a,b,c] = dcm2angle(vec2rot(pop(j,:)'));
        temp(j,:) = 180*abs([a,b,c])/pi;
    end
    RVar(i,:) = std(temp,0,1);
    
    [a,b,c] = dcm2angle(vec2rot(RErr(i,:)'));
    RErr(i,:) = 180*abs([a,b,c])/pi;
    
    [a,b,c] = dcm2angle(vec2rot(RErrEqual(i,:)'));
    RErrEqual(i,:) = 180*abs([a,b,c])/pi;
end

x = (10:10:1000)';

addpath('./plotBounds');

%plot
close all
figure
hold on;

subplot(3,1,3);
boundedline(x,RErr(:,1),RVar(:,1),'-b');
plot(x,RErrEqual(:,1),'-k');
title('Yaw Error');
axis([10 1000 0 5]);
set(gca,'layer','top');

subplot(3,1,2);
boundedline(x,RErr(:,2),RVar(:,2),'-r');
plot(x,RErrEqual(:,2),'-k');
axis([10 1000 0 5]);
set(gca,'layer','top');
title('Pitch Error');

subplot(3,1,1);
boundedline(x,RErr(:,3),RVar(:,3),'-g');
plot(x,RErrEqual(:,3),'-k');
axis([10 1000 0 5]);
set(gca,'layer','top');
title('Roll Error');

set(gcf,'color','w');

figure
hold on;

subplot(3,1,1);
boundedline(x,TErr(:,1),TVar(:,1),'-b');
plot(x,TErrEqual(:,1),'-k');
title('X Error');
axis([10 1000 0 3]);
set(gca,'layer','top');

subplot(3,1,2);
boundedline(x,TErr(:,2),TVar(:,2),'-r');
plot(x,TErrEqual(:,2),'-k');
axis([10 1000 0 3]);
set(gca,'layer','top');
title('Y Error');

subplot(3,1,3);
boundedline(x,TErr(:,3),TVar(:,3),'-g');
plot(x,TErrEqual(:,3),'-k');
axis([10 1000 0 3]);
set(gca,'layer','top');
title('Z Error');

set(gcf,'color','w');



