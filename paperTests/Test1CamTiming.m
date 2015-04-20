function [ err, var ] = Test1CamTiming( sensorData, offsetMag, timeSamples )
%TEST1CAMTIMING Summary of this function goes here
%   Detailed explanation goes here

baseOffset = 2*offsetMag*1000000*(rand(length(sensorData),1)-0.5);
baseOffset(1) = 0;

%add timing offset
for i = 1:length(sensorData)
    sensorData{i}.time = sensorData{i}.time + baseOffset(i);
end

% fix timestamps
[~, offsets, var] = CorrectTimestamps(sensorData, timeSamples);

%find error
var = var(:,1)/(1000000*1000000);
err = (offsets(:,1) - baseOffset)/1000000;

end

