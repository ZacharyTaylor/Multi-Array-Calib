function [] = plotSig( sensorData )
%PLOTSIG Summary of this function goes here
%   Detailed explanation goes here

addpath('./plotBounds');

Mag = zeros(size(sensorData{1}.T_Skm1_Sk,1),size(sensorData,1));
Sig = Mag;

for i = 1:size(sensorData,1)
    Mag(:,i) = sqrt(sum(sensorData{i}.T_Skm1_Sk(:,4:6).^2,2));
    Sig(:,i) = sqrt(sum(sensorData{i}.T_Cov_Skm1_Sk(:,4:6),2));
end

Mag = Mag(2:end,:);
Sig = Sig(2:end,:);

close all;
figure;
hold on;

%xlim([3092.23524159824 3433.66827172553]);
%ylim([-0.00953793171142787 0.0729944242012171]);

% Create xlabel
xlabel('Time (s)','FontSize',26);
% Create ylabel
ylabel('Rotation Magnitude (radians)','FontSize',26);

set(gca,'FontSize',20)
boundedline(1:size(Mag,1),Mag(:,1),3*Sig(:,1),'b-',1:size(Mag,1),Mag(:,2),3*Sig(:,2),'r-');%,'alpha');
%boundedline(1:size(Mag,1),Mag(:,2),3*Sig(:,2),'r-');%,'alpha');
