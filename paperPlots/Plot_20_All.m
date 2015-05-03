addpath('../tforms');

%load the data
data = load('../results/Test_21.1_Kitti.mat');

velCam1 = [0.272366992403569,0.000401046031637955,-0.0743065916413621,-1.21456402386260,1.18928496715603,-1.20243957469666];
velCam2 = [0.277752531522258,-0.536729003952002,-0.0776835216236700,-1.19271705822817,1.22722093116592,-1.20702757977384];
velCam3 = [0.269025963816496,0.0599294920311864,-0.0745533295124208,-1.21875939088579,1.19261583495551,-1.20811422737780];
velCam4 = [0.274374681029614,-0.472760662203873,-0.0751478768336082,-1.19212577481800,1.21673693585499,-1.20669925715255];

gt = [velCam1;velCam2;velCam3;velCam4];

fail = false(100,1);
res = zeros(100,6);
sd = zeros(100,6);
t = zeros(100,2);
for i = 1:100
    res(i,:) = mean(abs(data.results{i}.final(2:end,:)-gt));
    if(any(abs(data.results{i}.final(:))>3))
        fail(i) = true;
    end
    
    t(i,:) = mean(data.results{i}.timeOffset);
    
    temp = zeros(4,3);
    for j = 1:4
        [temp(j,1),temp(j,2),temp(j,3)] = dcm2angle(V2R(data.results{i}.final(j+1,4:6))/V2R(gt(j,4:6)));
    end
    res(i,4:6) = mean(abs(temp)*180/pi);
    
    sd(i,:) = sqrt(mean(data.results{i}.finalVar(2:end,:)));
    temp = zeros(4,3);
    for j = 1:4
        [~,temp(j,:)] = varChange(data.results{i}.final(j+1,4:6),sqrt(data.results{i}.finalVar(j+1,4:6)),gt(j,4:6));
    end
    
    sd(i,4:6) = mean(temp);
end

res = res(~fail,:);
sd = sd(~fail,:);

%% plot

subplot(3,1,1);
errorbar(res(:,1),sd(:,1),'ro')
axis([0,100,0,0.3])
title('X Error');

subplot(3,1,2);
errorbar(res(:,2),sd(:,2),'go')
axis([0,100,0,0.3])
title('Y Error');
ylabel('Calibration Error (m)');

subplot(3,1,3);
errorbar(res(:,3),sd(:,3),'bo')
axis([0,100,0,0.3])
xlabel('Run');
title('Z Error');

figure
subplot(3,1,1);
errorbar(res(:,4),sd(:,4),'ro')
axis([0,100,0,2])
title('Roll Error');

subplot(3,1,2);
errorbar(res(:,5),sd(:,5),'go')
axis([0,100,0,2])
ylabel('Calibration Error (degrees)');
title('Yaw Error');

subplot(3,1,3);
errorbar(res(:,6),sd(:,6),'bo')
axis([0,100,0,2])
xlabel('Run');
title('Pitch Error');