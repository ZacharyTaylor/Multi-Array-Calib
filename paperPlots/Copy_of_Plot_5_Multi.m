addpath('../tforms');

clear all
%data = {};

%load the data
data = load('../results/Test_5.1_Kitti.mat');

%ground truths
velNav = [-0.808675900000000,0.319555900000000,-0.799723100000000,0.0148243146805919,-0.00203019196358444,-0.000770383725406773];
velCam1 = [0.291804720076481,-0.0114055066485929,-0.0562394126383353,-1.21996291041592,1.19352064861705,-1.20328698135780];
velCam2 = [0.292034456780211,-0.548572251398307,-0.0605822662128782,-1.19838445835617,1.23177493496882,-1.20866875098245];

gt = [[0,0,0,0,0,0];velNav;velCam1;velCam2];

for i = [2,3,4]
    for j = 1:length(data.results)
        Tc1 = V2T([data.results{j}.tran(1,1:3),data.results{j}.rot(1,1:3)]);
        Tc2 = V2T([data.results{j}.tran(i,1:3),data.results{j}.rot(i,1:3)]);
        T = Tc2/Tc1;
        T = T / (V2T(gt(i,:))/V2T(gt(1,:)));
        err2(j,1:6,i) = abs(T2V(T));
    end
end

%% plot data


err2 = mean(err2,3);

for i = 1:size(err2,1)
    [err2(i,4:6),~] = varChange(err2(i,4:6),[0,0,0],[0,0,0]);
end

rot = ['Roll ';'Pitch';'Yaw  '];
tran = ['X';'Y';'Z'];

subplot(2,1,2);
boxplot([err2(:,1),err2(:,2),err2(:,3)],tran);
title('Translation error')
ylabel('Error (m)');

subplot(2,1,1);
boxplot([err2(:,4),err2(:,5),err2(:,6)],rot);
title('Rotation error')
ylabel('Error (degrees)');

