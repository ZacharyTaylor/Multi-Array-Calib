addpath('../tforms');

%load the data
data = load('../results/Test_24.2_Shrimp.mat');

fail = false(100,1);
res = zeros(100,6);
sd = zeros(100,6);
t = zeros(100,2);

r = 1;

for i = 1:7
    temp = data.results{i}.final(2:end,:);
    res(i,:) = mean(temp,1);
       
    t(i,:) = mean(data.results{i}.timeOffset);
    
    [r,p,y] = dcm2angle(V2R(data.results{i}.final(2,4:6)));
    res(i,4:6) = [r,p,y]*180/pi;
    
    sd(i,:) = sqrt(mean(data.results{i}.finalVar(2:end,:)));
    [~,sd(i,4:6)] = varChange(data.results{i}.final(2,4:6),sqrt(data.results{i}.finalVar(2,4:6)),[0,0,0]);
end

res = res(~fail,:);
sd = sd(~fail,:);

mean(res)
%% plot

subplot(3,1,1);
errorbar(res(:,1),sd(:,1),'ro')
%axis([0,100,0,0.5])
title('X Error');

subplot(3,1,2);
errorbar(res(:,2),sd(:,2),'go')
%axis([0,100,0,0.5])
title('Y Error');
ylabel('Calibration Error (m)');

subplot(3,1,3);
errorbar(res(:,3),sd(:,3),'bo')
%axis([0,100,0,0.5])
xlabel('Run');
title('Z Error');

figure
subplot(3,1,1);
errorbar(res(:,4),sd(:,4),'ro')
%axis([0,100,0,2])
title('Roll Error');

subplot(3,1,2);
errorbar(res(:,5),sd(:,5),'go')
%axis([0,100,0,2])
ylabel('Calibration Error (degrees)');
title('Yaw Error');

subplot(3,1,3);
errorbar(res(:,6),sd(:,6),'bo')
%axis([0,100,0,2])
xlabel('Run');
title('Pitch Error');