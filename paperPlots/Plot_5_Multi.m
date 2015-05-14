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

for i = [1,2,4]
    for j = 1:length(data.results)
        Tc1 = V2T([data.results{j}.tran(3,1:3),data.results{j}.rot(3,1:3)]);
        Tc2 = V2T([data.results{j}.tran(i,1:3),data.results{j}.rot(i,1:3)]);
        T = Tc2\Tc1;
        T = T / (V2T(gt(i,:))\V2T(gt(3,:)));
        err(j,1:6,i) = abs(T2V(T));

        Tc1 = V2T([data.results{j}.tranI(3,1:3),data.results{j}.rotI(3,1:3)]);
        Tc2 = V2T([data.results{j}.tranI(i,1:3),data.results{j}.rotI(i,1:3)]);
        T = Tc2\Tc1;
        T = T / (V2T(gt(i,:))\V2T(gt(3,:)));
        errI(j,1:6,i) = abs(T2V(T));
    end
end

for i = [2,3,4]
    for j = 1:length(data.results)
        Tc1 = V2T([data.results{j}.tran(1,1:3),data.results{j}.rot(1,1:3)]);
        Tc2 = V2T([data.results{j}.tran(i,1:3),data.results{j}.rot(i,1:3)]);
        T = Tc2\Tc1;
        T = T / (V2T(gt(i,:))\V2T(gt(1,:)));
        err2(j,1:6,i) = abs(T2V(T));

        Tc1 = V2T([data.results{j}.tranI(1,1:3),data.results{j}.rotI(1,1:3)]);
        Tc2 = V2T([data.results{j}.tranI(i,1:3),data.results{j}.rotI(i,1:3)]);
        T = Tc2\Tc1;
        T = T / (V2T(gt(i,:))\V2T(gt(1,:)));
        errI2(j,1:6,i) = abs(T2V(T));
    end
end

%% plot data

err = mean(err,3);
errI = mean(errI,3);

err2 = mean(err2,3);
errI2 = mean(errI2,3);

% rotErr = mean(rotErr,3);
% rotIErr = mean(rotIErr,3);
% tranErr = mean(tranErr,3);
% tranIErr = mean(tranIErr,3);

for i = 1:size(err,1)
    [errI(i,4:6),~] = varChange(errI(i,4:6),[0,0,0],[0,0,0]);
    [err(i,4:6),~] = varChange(err(i,4:6),[0,0,0],[0,0,0]);
    [errI2(i,4:6),~] = varChange(errI2(i,4:6),[0,0,0],[0,0,0]);
    [err2(i,4:6),~] = varChange(err2(i,4:6),[0,0,0],[0,0,0]);
end

rot = ['Roll comb ';'Roll sep  ';'Pitch comb';'Pitch sep ';'Yaw comb  ';'Yaw sep   '];
tran = ['X comb';'X sep ';'Y comb';'Y sep ';'Z comb';'Z sep '];

subplot(2,2,1)
boxplot([err2(:,1),errI2(:,1),err2(:,2),errI2(:,2),err2(:,3),errI2(:,3)],tran);
title('Translation error wrt Velodyne')
ylabel('Error (m)');
subplot(2,2,2)
boxplot([err2(:,4),errI2(:,4),err2(:,5),errI2(:,5),err2(:,6),errI2(:,6)],rot);
title('Rotation error wrt Velodyne')
ylabel('Error (degrees)');

subplot(2,2,3)
boxplot([err(:,1),errI(:,1),err(:,2),errI(:,2),err(:,3),errI(:,3)],tran);
title('Translation error wrt Leftmost camera')
ylabel('Error (m)');
subplot(2,2,4)
boxplot([err(:,4),errI(:,4),err(:,5),errI(:,5),err(:,6),errI(:,6)],rot);
title('Rotation error wrt Leftmost camera')
ylabel('Error (degrees)');