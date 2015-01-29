function [ sensorData ] = addOffset( sensorData, samples )
%ADDOFFSET Summary of this function goes here
%   Detailed explanation goes here

%% find transformations

T = cell(length(sensorData),1);
t = cell(length(sensorData),1);
for i = 1:length(sensorData)
    [~,idx] = sort(sensorData{i}.time);
    valid = diff(double(sensorData{i}.time(idx))) > 0;
    idx = idx(valid);
    t{i} = double(sensorData{i}.time(idx));
    
    %get absolute angle magnitude
    T{i} = sqrt(sum(sensorData{i}.T_S1_Sk(idx,4:6).^2,2));
end

%offset = zeros(length(sensorData),1);
offset = findErr2(T,t,samples);
for i = 1:length(sensorData)
    t{i} = t{i} - offset(i);
end

%get interpolation points
tMin = 0;
tMax = inf;
for i = 1:length(t)
    tMin = max(tMin,t{i}(1));
    tMax = min(tMax,t{i}(end));
end

tI = tMin:(tMax-tMin)/(samples-1):tMax;

for i = 1:length(sensorData)
    sensorData{i} = matchTformsTime(sensorData{i}, tI);
end

