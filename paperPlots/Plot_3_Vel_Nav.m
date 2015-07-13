addpath('../tforms');

%load the data
%data = load('../results/Test_3.10_Kitti.mat');
data = load('../results/Test_3.12_Shrimp.mat');

%ground truths
Kitti = [-0.808675900000000,0.319555900000000,-0.799723100000000,0.0148243146805919,-0.00203019196358444,-0.000770383725406773];
Shrimp = [-0.0319505121845316,-0.00484516177500113,0.882215281221151,0.0135769367640242,0.00199511274632100,-3.11427612111077];

gt = Shrimp;

valsRange = 1:500;

rotSD = zeros(length(valsRange),3,length(data.results{1}));
rotErr = zeros(length(valsRange),3,length(data.results{1}));
tranSD = zeros(length(valsRange),3,length(data.results{1}));
tranErr = zeros(length(valsRange),3,length(data.results{1}));
rotRErr = zeros(length(valsRange),3,length(data.results{1}));
tranRErr = zeros(length(valsRange),3,length(data.results{1}));

for j = valsRange
    for k = 1:length(data.results{j})

        rotSD(j,1:3,k) = sqrt(data.results{j}{k}.rotVar(2,1:3));
        rotErr(j,1:3,k) = abs(R2V(V2R(data.results{j}{k}.rot(2,1:3))/V2R(gt(4:6))));
        tranSD(j,1:3,k) = sqrt(data.results{j}{k}.tranVar(2,1:3));
        tranErr(j,1:3,k) = abs(data.results{j}{k}.tran(2,1:3) - gt(1:3));
        
        %get simple offset error
        rotRErr(j,1:3,k) = abs(R2V(V2R(data.results{j}{k}.rotR(2,1:3))/V2R(gt(4:6))));
        tranRErr(j,1:3,k) = abs(data.results{j}{k}.tranR(2,1:3) - gt(1:3));
    end
end

temp = rotSD;
rotSD = mean(rotSD);
for i = 1:size(temp,2)
    for j = 1:size(temp,3)
        rotSD(1,i,j) = mean(temp(isfinite(temp(:,i,j)),i,j));
    end
end
rotSD = reshape(rotSD,3,[])';
%rotErr = abs(rotErr - repmat(mean(rotErr),size(rotErr,1),1));
rotErr = mean(rotErr,1);
rotErr = reshape(rotErr,3,[])';
%rotRErr = abs(rotRErr - repmat(mean(rotRErr),size(rotRErr,1),1));
rotRErr = mean(rotRErr,1);
rotRErr = reshape(rotRErr,3,[])';

temp = tranSD;
tranSD = mean(tranSD);
for i = 1:size(temp,2)
    for j = 1:size(temp,3)
        tranSD(1,i,j) = mean(temp(isfinite(temp(:,i,j)),i,j));
    end
end
tranSD = reshape(tranSD,3,[])';
%tranErr = abs(tranErr - repmat(mean(tranErr),size(tranErr,1),1));
tranErr = mean(tranErr,1);
tranErr = reshape(tranErr,3,[])';
%tranRErr = abs(tranRErr - repmat(mean(tranRErr),size(tranRErr,1),1));
tranRErr = mean(tranRErr,1);
tranRErr = reshape(tranRErr,3,[])';

for i = 1:size(rotErr,1)
    [rotErr(i,:),rotSD(i,:)] = varChange(rotErr(i,:),rotSD(i,:),[0,0,0]);
    [rotRErr(i,:),~] = varChange(rotRErr(i,:),[0,0,0],[0,0,0]);
end

%% plot results

figure
subplot(3,2,1);
hold on;
boundedline((10:10:300),rotErr(:,1),rotSD(:,1),'ro-');
plot((10:10:300),rotRErr(:,1),'kx-');
axis([0,300,0,2])
title('Roll Error');

subplot(3,2,3);
hold on;
boundedline((10:10:300),rotErr(:,2),rotSD(:,2),'go-');
plot((10:10:300),rotRErr(:,2),'kx-');
axis([0,300,0,2])
title('Pitch Error');
ylabel('Calibration Error (degrees)');

subplot(3,2,5);
hold on;
boundedline((10:10:300),rotErr(:,3),rotSD(:,3),'bo-');
plot((10:10:300),rotRErr(:,3),'kx-');
axis([0,300,0,2])
title('Yaw Error');
xlabel('Length of data used (seconds)');

subplot(3,2,2);
hold on;
boundedline((10:10:300),tranErr(:,1),tranSD(:,1),'ro-');
plot((10:10:300),tranRErr(:,1),'kx-');
axis([0,300,0,0.5])
title('X Error');

subplot(3,2,4);
hold on;
boundedline((10:10:300),tranErr(:,2),tranSD(:,2),'go-');
plot((10:10:300),tranRErr(:,2),'kx-');
axis([0,300,0,0.5])
title('Y Error');
ylabel('Calibration Error (m)');

subplot(3,2,6);
hold on;
boundedline((10:10:300),tranErr(:,3),tranSD(:,3),'bo-');
plot((10:10:300),tranRErr(:,3),'kx-');
axis([0,300,0,0.5])
title('Z Error');
xlabel('Length of data used (seconds)');
