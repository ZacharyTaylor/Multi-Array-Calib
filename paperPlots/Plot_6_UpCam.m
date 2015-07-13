addpath('../tforms');

%clear all
data = {};

%load the data
data{1} = load('../results/Test_6.1.mat');
data{2} = load('../results/Test_6.2.mat');
data{3} = load('../results/Test_6.3.mat');
data{4} = load('../results/Test_6.4.mat');
data{5} = load('../results/Test_6.5.mat');
data{6} = load('../results/Test_6.6.mat');
data{7} = load('../results/Test_6.7.mat');
data{8} = load('../results/Test_6.8.mat');
data{9} = load('../results/Test_6.9.mat');

%ground truths
cam1Cam6 = [1.57699318480674,0.00313558970979745,0.00786411362258004];

gt = V2R(cam1Cam6);

err = zeros(length(data{1}.results),3,9);
v = err;
for i = 1:length(data{1}.results)
    for j = 1:9
        try
            R = V2R(data{j}.results{i}.rot(end,:));
            R = R \ gt;
            v(i,:,j) = sqrt(data{j}.results{i}.rotVar(end,:));
            [v(i,1,j),v(i,2,j),v(i,3,j)] = dcm2angle(V2R(v(i,:,j)));
            v(i,:,j) = abs(v(i,:,j))*180/pi;

            [err(i,1,j),err(i,2,j),err(i,3,j)] = dcm2angle(R);
            err(i,:,j) = abs(err(i,:,j))*180/pi;
        catch
            v(i,:,j) = nan;
            err(i,:,j) = nan;
        end
    end
end

%% plot data
figure;
subplot(3,1,1)
boxplot([err(:,1,4),err(:,1,1),err(:,1,7),err(:,2,4),err(:,2,1),err(:,2,7),err(:,3,4),err(:,3,1),err(:,3,7)],{'Roll 75%','Roll 25%','Roll 0%','Pitch 75%','Pitch 25%','Pitch 0%','Yaw 75%','Yaw 25%','Yaw 0%'});
title('20 Seconds of Data');

subplot(3,1,2)
boxplot([err(:,1,5),err(:,1,2),err(:,1,8),err(:,2,5),err(:,2,2),err(:,2,8),err(:,3,5),err(:,3,2),err(:,3,8)],{'Roll 75%','Roll 25%','Roll 0%','Pitch 75%','Pitch 25%','Pitch 0%','Yaw 75%','Yaw 25%','Yaw 0%'});
ylabel('Rotational Error (degrees)');
title('100 Seconds of Data');

subplot(3,1,3)
boxplot([err(:,1,6),err(:,1,3),err(:,1,9),err(:,2,6),err(:,2,3),err(:,2,9),err(:,3,6),err(:,3,3),err(:,3,9)],{'Roll 75%','Roll 25%','Roll 0%','Pitch 75%','Pitch 25%','Pitch 0%','Yaw 75%','Yaw 25%','Yaw 0%'});
title('200 Seconds of Data');

err = v;

figure;
subplot(3,1,1)
boxplot([err(:,1,4),err(:,1,1),err(:,1,7),err(:,2,4),err(:,2,1),err(:,2,7),err(:,3,4),err(:,3,1),err(:,3,7)],{'Roll 75%','Roll 25%','Roll 0%','Pitch 75%','Pitch 25%','Pitch 0%','Yaw 75%','Yaw 25%','Yaw 0%'});
title('20 Seconds of Data');

subplot(3,1,2)
boxplot([err(:,1,5),err(:,1,2),err(:,1,8),err(:,2,5),err(:,2,2),err(:,2,8),err(:,3,5),err(:,3,2),err(:,3,8)],{'Roll 75%','Roll 25%','Roll 0%','Pitch 75%','Pitch 25%','Pitch 0%','Yaw 75%','Yaw 25%','Yaw 0%'});
ylabel('Rotational Standard Deviation (degrees)');
title('100 Seconds of Data');

subplot(3,1,3)
boxplot([err(:,1,6),err(:,1,3),err(:,1,9),err(:,2,6),err(:,2,3),err(:,2,9),err(:,3,6),err(:,3,3),err(:,3,9)],{'Roll 75%','Roll 25%','Roll 0%','Pitch 75%','Pitch 25%','Pitch 0%','Yaw 75%','Yaw 25%','Yaw 0%'});
title('200 Seconds of Data');