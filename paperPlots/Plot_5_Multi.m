addpath('../tforms');

%data = {};

%load the data
data = load('../results/Test_4.1_Kitti.mat');

%ground truths
%Kitti = [0.272366992403569,0.000401046031637955,-0.0743065916413621,-1.21456402386260,1.18928496715603,-1.20243957469666];
Kitti = [1.08275954635987,-0.307549204234380,0.727865316091136,-1.22740819019080,1.18319377481942,-1.21165360963875];
gt = Kitti;

for j = 1:80
    for k = 1:length(data.results{j})

        rotSD(j,1:3,k) = sqrt(data.results{j}{k}.rotVar(2,1:3));
        rotErr(j,1:3,k) = abs(R2V(V2R(data.results{j}{k}.rot(2,1:3))/V2R(gt(4:6))));
        %rotErr(j,1:3,k) = abs(data.results{j}{k}.rot(2,1:3) - gt(4:6));
        tranSD(j,1:3,k) = sqrt(data.results{j}{k}.tranVar(2,1:3));
        tranErr(j,1:3,k) = abs(data.results{j}{k}.tran(2,1:3) - gt(1:3));
        
        %get simple offset error
        rotRErr(j,1:3,k) = abs(data.results{j}{k}.rotR(2,1:3) - gt(4:6));
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
rotErr = mean(rotErr,1);
rotErr = reshape(rotErr,3,[])';
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
tranErr = mean(tranErr,1);
tranErr = reshape(tranErr,3,[])';
tranRErr = mean(tranRErr,1);
tranRErr = reshape(tranRErr,3,[])';

for i = 1:size(rotErr,1)
    [rotErr(i,:),rotSD(i,:)] = varChange(rotErr(i,:),rotSD(i,:));
    [rotRErr(i,:),~] = varChange(rotRErr(i,:),[0,0,0]);
end

%% plot results

figure
subplot(3,2,1);
hold on;
boundedline((10:10:200),rotErr(:,1),rotSD(:,1),'ro-');
plot((10:10:200),rotRErr(:,1),'kx-');
axis([0,200,0,5])
title('Roll Error');

subplot(3,2,3);
hold on;
boundedline((10:10:200),rotErr(:,2),rotSD(:,2),'go-');
plot((10:10:200),rotRErr(:,2),'kx-');
axis([0,200,0,2])
title('Pitch Error');
ylabel('Calibration Error (degrees)');

subplot(3,2,5);
hold on;
boundedline((10:10:200),rotErr(:,3),rotSD(:,3),'bo-');
plot((10:10:200),rotRErr(:,3),'kx-');
axis([0,200,0,2])
title('Yaw Error');
xlabel('Run');

subplot(3,2,2);
hold on;
boundedline((10:10:200),tranErr(:,1),tranSD(:,1),'ro-');
plot((10:10:200),tranRErr(:,1),'kx-');
axis([0,200,0,2])
title('X Error');

subplot(3,2,4);
hold on;
boundedline((10:10:200),tranErr(:,2),tranSD(:,2),'go-');
plot((10:10:200),tranRErr(:,2),'kx-');
axis([0,200,0,2])
title('Y Error');
ylabel('Calibration Error (m)');

subplot(3,2,6);
hold on;
boundedline((10:10:200),tranErr(:,3),tranSD(:,3),'bo-');
plot((10:10:200),tranRErr(:,3),'kx-');
axis([0,200,0,2])
title('Z Error');
xlabel('Run');

