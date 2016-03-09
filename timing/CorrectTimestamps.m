function [ sensorData, offsets, varOff ] = CorrectTimestamps( sensorData, samples, applyOffset )
%CORRECTTIMESTAMPS correct for any offset in intersensor timesteps
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   sensorData- nx1 cell containing sensor data sturcts
%   samples - scalar, number of points to sample the data at (uniformly
%       distributed) when finding timestamps
%   applyOffset - bool, if the found timing offsets should be applied
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
    [~,idx] = sort(sensorData{i}.times);
    
    %remove points with same timestamp
    valid = diff(double(sensorData{i}.times(idx))) > 0;
    idx = idx(valid);
    
    %sort sensorData
    sensorData{i} = SensorDataSubset(sensorData{i}, idx);
end

%preallocate memory
Mag = cell(length(sensorData),1);
Var = cell(length(sensorData),1);
t = cell(length(sensorData),1);
navFlag = false(length(sensorData),1);

%find absolute angle magintude and variance
for i = 1:length(sensorData)
    
    %get timestamps
    t{i} = double(sensorData{i}.times);
    
    %get absolute angle magnitude
    Mag{i} = sqrt(sum(sensorData{i}.T_S1_Sk(:,4:6).^2,2));
    Var{i} = sum(sensorData{i}.T_Var_S1_Sk(:,4:6),2);
    navFlag(i) = strcmpi(sensorData{i}.type,'Nav');
end

%find the timing offsets
[offsets, varOff] = FindTimingOffsets(Mag,Var,t,samples, navFlag);
offsets(:,1) = 0;
offsets(:,2) = 1;
varOff(:) = 0;

%apply offsets
for i = 1:length(sensorData)
    sensorData{i}.times = double(sensorData{i}.times);
    sensorData{i}.times = sensorData{i}.times - offsets(i,1);
    tMin = min(sensorData{i}.times);
    sensorData{i}.times = sensorData{i}.times - tMin;
    sensorData{i}.times = sensorData{i}.times.*offsets(i,2);
    sensorData{i}.times = sensorData{i}.times + tMin;
end

%apply offset variance
for i = 1:length(sensorData)
    err = (sqrt(varOff(i,1))./median(diff(double(sensorData{i}.times)))).*((sensorData{i}.T_Skm1_Sk));
    sensorData{i}.T_Var_Skm1_Sk = sensorData{i}.T_Var_Skm1_Sk + err.^2;
    if(navFlag(i))
        sensorData{i}.T_Var_S1_Sk = sensorData{i}.T_Var_Skm1_Sk;
    else
        sensorData{i}.T_Var_S1_Sk = cumsum(sensorData{i}.T_Var_Skm1_Sk);
    end
end

end

