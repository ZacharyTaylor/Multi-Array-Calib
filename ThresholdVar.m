function [ sensorData, valid ] = ThresholdVar( sensorData, threshold )
%THRESHOLDVAR Removes points with variance greater then threshold, negitive
%   variance, imaginary variance and non finite variance
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   sensorData- nx1 cell containing sensor data sturcts
%   threshold- maximum allowable variance
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   sensorData- nx1 cell containing sensor data sturcts
%   valid- mx1 index of acceptable points
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
validateattributes(threshold,{'numeric'},{'scalar','positive'});

valid = true(size(sensorData{1}.T_Var_Skm1_Sk,1),1);

%get points where 0 < variance < threshold
for i = 1:size(sensorData,1)
    valid = and(valid,and(all(sensorData{i}.T_Var_Skm1_Sk > 0,2),all(sensorData{i}.T_Var_Skm1_Sk < threshold,2)));
end

%get imagniary points
for i = 1:size(sensorData,1)
    valid = and(valid,all(imag(sensorData{i}.T_Skm1_Sk) == 0,2));
end

%get nonfinite points
for i = 1:size(sensorData,1)
    valid = and(valid,all(isfinite(sensorData{i}.T_Skm1_Sk),2));
end

%get negitive scale
for i = 1:size(sensorData,1)
    valid = and(valid,and(sensorData{i}.T_Skm1_Sk(:,4) > 0,sensorData{i}.T_Skm1_Sk(:,4) < threshold));
end

%turn valid into index
valid = find(valid);

%remove invalid points
sensorData = SensorDataSubset(sensorData, valid);
