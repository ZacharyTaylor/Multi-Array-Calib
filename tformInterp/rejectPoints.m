function [ sensorData ] = rejectPoints( sensorData )
%REJECTPOINTS Summary of this function goes here
%   Detailed explanation goes here

Mag = zeros(size(sensorData{1}.T_Skm1_Sk,1),size(sensorData,1));
Sig = Mag;

for i = 1:size(sensorData,1)
    Mag(:,i) = sqrt(sum(sensorData{i}.T_Skm1_Sk(:,4:6).^2,2));
    Sig(:,i) = sum(sensorData{i}.T_Cov_Skm1_Sk(:,4:6),2);
end

iSig = 1./Sig;

mMag = sum(Mag.*iSig,2)./sum(iSig,2);
Sig = sqrt(Sig);

valid = all(abs(repmat(mMag,1,size(Mag,2)) - Mag) < 3*Sig,2);
valid = and(valid,all(Sig < 1,2));

for i = 1:size(sensorData,1)
    sensorData{i}.T_Skm1_Sk = sensorData{i}.T_Skm1_Sk(valid,:);
    sensorData{i}.T_S1_Sk = sensorData{i}.T_S1_Sk(valid,:);
    sensorData{i}.T_Cov_Skm1_Sk = sensorData{i}.T_Cov_Skm1_Sk(valid,:);
    sensorData{i}.time = sensorData{i}.time(valid,:);
    sensorData{i}.files = sensorData{i}.files(valid,:);
end
