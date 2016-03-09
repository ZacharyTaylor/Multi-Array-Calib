function [ sensorData ] = SampleData( sensorData )
%SAMPLEDATA2 Uniformly samples data at rate of slowest sensor
%--------------------------------------------------------------------------
%   Inputs:
%--------------------------------------------------------------------------
%   sensorData- nx1 cell containing sensor data sturcts
%   samples - scalar, number of points to sample the data at (uniformly
%       distributed)
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   sensorData- nx1cell containing sensor data sturcts
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

%check inputs
validateattributes(sensorData,{'cell'},{'vector'});
for i = 1:length(sensorData)
    validateattributes(sensorData{i},{'struct'},{});
end

addpath('./timing');

%get interpolation points
tMin = 0;
tMax = inf;
for i = 1:length(sensorData)
    tMin = max(tMin,sensorData{i}.times(1));
    tMax = min(tMax,sensorData{i}.times(end));
end

%get number of points in range
samples = inf;
for i = 1:length(sensorData)
    temp = sum(and(sensorData{i}.times >= tMin, sensorData{i}.times <= tMax));
    if(temp < samples)
        samples = temp;
    end
end

%turn points into times
times = tMin:(tMax-tMin)/(samples):tMax;

%interpolate at set times
sensorData = IntSensorData(sensorData, times);

end
