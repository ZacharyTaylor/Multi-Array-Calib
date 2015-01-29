function [ sensorData ] = removeStaticPoints( sensorData )
%REJECTPOINTS Summary of this function goes here
%   Detailed explanation goes here

for i = 1:size(sensorData,1)
    
    Mag = sqrt(sum(sensorData{i}.T_Skm1_Sk(:,4:6).^2,2));
    
    t = diff(double(sensorData{i}.time));
    t = [t(1);t]/1000000;

    %require minimum rotation
    valid = all((Mag./t) > 0.001,2);

    sensorData{i}.T_Skm1_Sk = sensorData{i}.T_Skm1_Sk(valid,:);
    sensorData{i}.T_S1_Sk = sensorData{i}.T_S1_Sk(valid,:);
    sensorData{i}.T_Cov_Skm1_Sk = sensorData{i}.T_Cov_Skm1_Sk(valid,:);
    sensorData{i}.time = sensorData{i}.time(valid,:);
    sensorData{i}.files = sensorData{i}.files(valid,:);
end
