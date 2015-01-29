function [ sensorData ] = randTformsTime( sensorData, timeLength )
%RANDTFORMS returns num contiguous transforms and transformsVar

timeLength = timeLength*1000000;
minT = sensorData{1}.time(1);
maxT = sensorData{1}.time(end);

%get random index to use for scans
idx = (maxT-minT-timeLength)*rand(1)  + minT;
idx = and(sensorData{1}.time >= idx, sensorData{1}.time <= (idx+timeLength));

%get corrosponding transformations
for i = 1:size(sensorData,1)
    sensorData{i}.T_Skm1_Sk = sensorData{i}.T_Skm1_Sk(idx,:);
    sensorData{i}.T_S1_Sk = sensorData{i}.T_S1_Sk(idx,:);
    sensorData{i}.T_Cov_Skm1_Sk = sensorData{i}.T_Cov_Skm1_Sk(idx,:);
    sensorData{i}.time = sensorData{i}.time(idx,:);
    sensorData{i}.files = sensorData{i}.files(idx,:);
end

end

