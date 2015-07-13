addpath('../tforms');

%load the data
data = load('./results/Test_9.1.mat');

%10_03
velCam1 = [0.291804720076481,-0.0114055066485929,-0.0562394126383353,-1.21437856632943,1.20575835034676,-1.20140449070730];
velCam2 = [0.292034456780211,-0.548572251398307,-0.0605822662128782,-1.19262401861693,1.24394317283683,-1.20667887644520];
velCam3 = [0.287988792734513,0.0481207596442812,-0.0572301603112048,-1.21927554848229,1.20959078381303,-1.20612910053213];
velCam4 = [0.287103485048149,-0.485304279180288,-0.0584847671355261,-1.19258234104363,1.23385881496779,-1.20595346907977];

gt = [velCam1;velCam2;velCam3;velCam4];

fail = false(100,1);
res = zeros(100,6);
sd = zeros(100,6);
t = zeros(100,2);

r = 1:4;

for i = 1:100
    
    [data.results{i}.final, data.results{i}.finalVar] = OptGrid(data.results{i}.ref, data.results{i}.vref);
    
    temp = abs(data.results{i}.final(2:end,:)-gt);
    res(i,:) = mean(temp(r,:),1);
    
    if(any(abs(data.results{i}.final(:))>3))
        fail(i) = true;
    end
       
    temp = zeros(4,3);
    for j = 1:4
        [temp(j,1),temp(j,2),temp(j,3)] = dcm2angle(V2R(data.results{i}.final(j+1,4:6))/V2R(gt(j,4:6)));
    end
    res(i,4:6) = mean(abs(temp(r,:))*180/pi,1);
    
    sd(i,:) = sqrt(mean(data.results{i}.finalVar(2:end,:)));
    temp = zeros(4,3);
    for j = 1:4
        [~,temp(j,:)] = varChange(data.results{i}.final(j+1,4:6),sqrt(data.results{i}.finalVar(j+1,4:6)),gt(j,4:6));
    end
    
    sd(i,4:6) = mean(temp(r,:),1);
end

res = res(~fail,:);
sd = sd(~fail,:);

mean(res)
%% plot

subplot(3,1,1);
MyErrorbars(res(:,1), sd(:,1), 'r')
axis([0,100,0,0.5])
title('X Error');

subplot(3,1,2);
MyErrorbars(res(:,2), sd(:,2), 'g')
axis([0,100,0,0.5])
title('Y Error');
ylabel('Calibration Error (m)');

subplot(3,1,3);
MyErrorbars(res(:,3), sd(:,3), 'b')
axis([0,100,0,0.5])
xlabel('Run');
title('Z Error');

figure
subplot(3,1,1);
MyErrorbars(res(:,4), sd(:,4), 'r')
axis([0,100,0,2])
title('Roll Error');

subplot(3,1,2);
MyErrorbars(res(:,5), sd(:,5), 'g')
axis([0,100,0,2])
ylabel('Calibration Error (degrees)');
title('Yaw Error');

subplot(3,1,3);
MyErrorbars(res(:,6), sd(:,6), 'b')
axis([0,100,0,2])
xlabel('Run');
title('Pitch Error');

figure
labels = {'X','Y','Z','Roll','Pitch','Yaw'};
boxplot(res./sd,labels);
ylabel('estimated std from ground truth');
