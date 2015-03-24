function [ outVec ] = ErrorEstR( sensorData, estVec, samples )
%ERRORESTR estimate error from variance in rotation vectors after alignment
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   sensorData- nx1 cell containing sensor data sturcts
%   estVec- nx3 matrix of rotations for each sensor
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   varVec- nx3 matrix containing rotational variance
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
validateattributes(estVec,{'numeric'},{'size',[length(sensorData),3]});

%refine rotation estimate and record result
options = optimset('MaxFunEvals',100000,'MaxIter',5000);

outVec = zeros([size(estVec),samples]);
for i = 1:samples
    %bootstrap data
    idx = datasample(1:size(sensorData{1}.time,1),size(sensorData{1}.time,1));
    sData = SensorDataSubset(sensorData, idx);
    outVec(2:end,:,i) = fminsearch(@(estVec) SystemProbR(sData, estVec),estVec(2:end,:), options);
end

outVec = var(outVec,[],3);

