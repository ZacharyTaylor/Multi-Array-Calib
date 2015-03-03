function [ sensorData, offsets ] = CorrectTimestamps( sensorData, samples )
%CORRECTTIMESTAMPS correct for any offset in intersensor timesteps
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   sensorData- nx1 cell containing sensor data sturcts
%   samples - scalar, number of points to sample the data at (uniformly
%       distributed) when finding timestamps
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   sensorData- nx1 cell containing sensor data sturcts with offsets
%       accounted for
%   offsets- nx1 vector containing the sensor time offsets
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

%sort sensorData by time
for i = 1:length(sensorData)
    %sort points in order of time they occoured
    [~,idx] = sort(sensorData{i}.time);
    
    %remove points with same timestamp
    valid = diff(double(sensorData{i}.time(idx))) > 0;
    idx = idx(valid);
    
    %sort sensorData
    sensorData{i} = SensorDataSubset(sensorData{i}, idx);
end

%preallocate memory
Mag = cell(length(sensorData),1);
Var = cell(length(sensorData),1);
t = cell(length(sensorData),1);

%find absolute angle magintude and variance
for i = 1:length(sensorData)
    
    %get timestamps
    t{i} = double(sensorData{i}.time);
    
    %get absolute angle magnitude
    Mag{i} = sqrt(sum(sensorData{i}.T_S1_Sk(:,5:7).^2,2));
    Var{i} = sqrt(sum(sensorData{i}.T_Var_S1_Sk(:,5:7).^2,2));
end

%find the timing offsets
offsets = FindTimingOffsets(Mag,Var,t,samples);

%apply offsets
for i = 1:length(sensorData)
    sensorData{i}.time = sensorData{i}.time - offsets(i);
end

end

