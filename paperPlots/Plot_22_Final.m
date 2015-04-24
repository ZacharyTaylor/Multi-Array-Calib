addpath('../tforms');

%load the data
data = load('../results/Test_2.1_Kitti.mat');

gt = [0.272366992403569,0.000401046031637955,-0.0743065916413621,-1.21456402386260,1.18928496715603,-1.20243957469666];

res = zeros(100,6);
sd = zeros(100,6);
for i = 1:100
    res(i,:) = abs(data.results{i}.final(2,:)-gt);
    
    [r,p,y] = dcm2angle(V2R(data.results{i}.final(2,4:6))/V2R(gt(4:6)));
    res(i,4:6) = abs([r,p,y])*180/pi;
    
    sd(i,:) = sqrt(data.results{i}.finalVar(2,:));
    sd(i,4:6) = varChange(data.results{i}.final(2,4:6),data.results{i}.finalVar(2,4:6),gt(4:6));
end

%% plot

subplot(3,1,1);
errorbar(res(:,1),sd(:,1),'ro')
axis([0,100,0,0.5])
title('X Error');

subplot(3,1,2);
errorbar(res(:,2),sd(:,2),'go')
axis([0,100,0,0.5])
title('Y Error');
ylabel('Calibration Error (m)');

subplot(3,1,3);
errorbar(res(:,3),sd(:,3),'bo')
axis([0,100,0,0.5])
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