function [ err ] = Test1CamTiming( sensorData, offsetMag, timeSamples )
%TEST1CAMTIMING Summary of this function goes here
%   Detailed explanation goes here

baseOffset = 2*offsetMag*1000000*(rand(length(sensorData),1)-0.5);
baseOffset(1) = 0;

%add timing offset
for i = 1:length(sensorData)
    sensorData{i}.time = sensorData{i}.time + baseOffset(i);
end

% fix timestamps
[~, offsets] = CorrectTimestamps(sensorData, timeSamples);

%find error
err = (offsets - baseOffset)/1000000;

end

