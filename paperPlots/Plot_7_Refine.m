addpath('../tforms');

%load the data
data = load('../results/Test_7.3.mat');
%data2 = load('../results/Test_7.2.mat');

num = 91;

%ground truths
gt = [0.291804720076481,-0.0114055066485929,-0.0562394126383353,-1.21996291041592,1.19352064861705,-1.20328698135780];

fail = [2,2,2,0.3,0.3,0.3];

inital = zeros(num,6);
nmi = zeros(num,6);
nmiC = zeros(num,6);
gom = zeros(num,6);
gomC = zeros(num,6);
lev = zeros(num,6);
levC = zeros(num,6);
motion = zeros(num,6);
motionC = zeros(num,6);

for i = 1:num
    tmp = [data.results{i}.tran(2,:),data.results{i}.rot(2,:)];
    T = V2T(tmp)/V2T(gt);
    [r,p,y] = dcm2angle(T(1:3,1:3));
    inital(i,:) = abs([[r,p,y]*180/pi,T(1:3,4)']);
    
%     tmp = [data.results{i}.tranVar(2,:),data.results{i}.rotVar(2,:)];
%     T = V2T(sqrt(tmp));
%     [r,p,y] = dcm2angle(T(1:3,1:3));
%     inital(i,:) = abs([[r,p,y]*180/pi,T(1:3,4)']);
    
    tmp = data.results{i}.NMIRef;
    T = V2T(tmp)/V2T(gt);
    [r,p,y] = dcm2angle(T(1:3,1:3));
    nmi(i,:) = abs([[r,p,y]*180/pi,T(1:3,4)']);
    
    tmp = data.results{i}.NMIConRef;
    T = V2T(tmp)/V2T(gt);
    [r,p,y] = dcm2angle(T(1:3,1:3));
    nmiC(i,:) = abs([[r,p,y]*180/pi,T(1:3,4)']);
    
    tmp = data.results{i}.LevRef;
    T = V2T(tmp)/V2T(gt);
    [r,p,y] = dcm2angle(T(1:3,1:3));
    lev(i,:) = abs([[r,p,y]*180/pi,T(1:3,4)']);
    
    tmp = data.results{i}.LevConRef;
    T = V2T(tmp)/V2T(gt);
    [r,p,y] = dcm2angle(T(1:3,1:3));
    levC(i,:) = abs([[r,p,y]*180/pi,T(1:3,4)']);
    
    tmp = data.results{i}.GOMRef;
    T = V2T(tmp)/V2T(gt);
    [r,p,y] = dcm2angle(T(1:3,1:3));
    gom(i,:) = abs([[r,p,y]*180/pi,T(1:3,4)']);
    
    tmp = data.results{i}.GOMconRef;
    T = V2T(tmp)/V2T(gt);
    [r,p,y] = dcm2angle(T(1:3,1:3));
    gomC(i,:) = abs([[r,p,y]*180/pi,T(1:3,4)']);
    
    tmp = data.results{i}.MotionRef;
    T = V2T(tmp)/V2T(gt);
    [r,p,y] = dcm2angle(T(1:3,1:3));
    motion(i,:) = abs([[r,p,y]*180/pi,T(1:3,4)']);
    
    tmp = data.results{i}.MotionConRef;
    T = V2T(tmp)/V2T(gt);
    [r,p,y] = dcm2angle(T(1:3,1:3));
    motionC(i,:) = abs([[r,p,y]*180/pi,T(1:3,4)']);
end

initalS = sum(any(inital > repmat(fail,num,1),2));
nmiS = sum(any(nmi > repmat(fail,num,1),2));
nmiCS = sum(any(nmiC > repmat(fail,num,1),2));
gomS = sum(any(gom > repmat(fail,num,1),2));
gomCS = sum(any(gomC > repmat(fail,num,1),2));
levS = sum(any(lev > repmat(fail,num,1),2));
levCS = sum(any(levC > repmat(fail,num,1),2));
motionS = sum(any(motion > repmat(fail,num,1),2));
motionCS = sum(any(motionC > repmat(fail,num,1),2));

labels = {'Inital','Lev','GOM','NMI','IM'};

%% plot data
figure;
subplot(3,2,1)
boxplot([inital(:,1),levC(:,1),gomC(:,1),nmiC(:,1),motionC(:,1)],labels);
ylim([0 2]);
title('Roll');

subplot(3,2,3)
boxplot([inital(:,2),levC(:,2),gomC(:,2),nmiC(:,2),motionC(:,2)],labels);
ylim([0 2]);
title('Pitch');

subplot(3,2,5)
boxplot([inital(:,3),levC(:,3),gomC(:,3),nmiC(:,3),motionC(:,3)],labels);
ylim([0 2]);
title('Yaw');

subplot(3,2,2)
boxplot([inital(:,4),levC(:,4),gomC(:,4),nmiC(:,4),motionC(:,4)],labels);
ylim([0 0.3]);
title('X');

subplot(3,2,4)
boxplot([inital(:,5),levC(:,5),gomC(:,5),nmiC(:,5),motionC(:,5)],labels);
ylim([0 0.3]);
title('Y');

subplot(3,2,6)
boxplot([inital(:,6),levC(:,6),gomC(:,6),nmiC(:,6),motionC(:,6)],labels);
ylim([0 0.3]);
title('Z');

figure;
subplot(3,2,1)
boxplot([inital(:,1),lev(:,1),gom(:,1),nmi(:,1),motion(:,1)],labels);
title('Roll');

subplot(3,2,3)
boxplot([inital(:,2),lev(:,2),gom(:,2),nmi(:,2),motion(:,2)],labels);
title('Pitch');

subplot(3,2,5)
boxplot([inital(:,3),lev(:,3),gom(:,3),nmi(:,3),motion(:,3)],labels);
title('Yaw');

subplot(3,2,2)
boxplot([inital(:,4),lev(:,4),gom(:,4),nmi(:,4),motion(:,4)],labels);
title('X');

subplot(3,2,4)
boxplot([inital(:,5),lev(:,5),gom(:,5),nmi(:,5),motion(:,5)],labels);
title('Y');

subplot(3,2,6)
boxplot([inital(:,6),lev(:,6),gom(:,6),nmi(:,6),motion(:,6)],labels);
title('Z');