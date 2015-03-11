function [ sensorData ] = RandTformTimes( sensorData, timeLength )
%RANDTFORMTIMES gets a random contiguous section of sensor data of length
%   timeLength
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   sensorData- a nx1 cell containing sensor data sturcts
%   timeLength- length of required data in seconds
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   sensorData- a nx1 cell containing sensor data sturcts
%
%--------------------------------------------------------------------------
%   References:
%--------------------------------------------------------------------------
%   This function is part of the Multi-Array-Calib toolbox 
%   https://github.com/ZacharyTaylor/Multi-Array-Calib
%   
%   This code was written by Zachary Taylor
%   zacharyjeremytaylor@gmail.com
%   http://www.zjtaylor.com

validateattributes(sensorData,{'cell'},{'vector'});
validateattributes(timeLength,{'numeric'},{'positive','scalar'});


timeLength = 1000000*timeLength;

startTime = 0;
endTime = inf;
for i = 1:length(sensorData)
    startTime = max(startTime,sensorData{i}.time(1));
    endTime = min(endTime,sensorData{i}.time(end));
end

dataLength = endTime - startTime;

if(timeLength > dataLength)
    error('Not enough data for set time');
end

startT = startTime + rand(1)*(dataLength-timeLength);
endT = startT + timeLength;

for i = 1:length(sensorData)
    valid = and(sensorData{i}.time > startT, sensorData{i}.time < endT);
    valid = find(valid);
    sensorData{i} = SensorDataSubset(sensorData{i}, valid);
end
    
end

