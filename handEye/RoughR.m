function [ estVec ] = RoughR( sensorData )
%ROUGHR finds an approximation to R using weighted least squares
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   sensorData- nx1 cell containing sensor data sturcts
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   estVec- nx3 matrix of the rotation for each sensor
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

estVec = zeros(size(sensorData,1),3);
for i = 2:size(sensorData,1)
    %takes maximum variance and convert to weighting
    weight = 1./sqrt((max(sensorData{1}.T_Var_Skm1_Sk(:,5:7),[],2) + max(sensorData{i}.T_Var_Skm1_Sk(:,5:7),[],2)));
    weight = weight(:)';
    
    %use Kabsch to find rotation estimate
    [temp,~] = Kabsch(sensorData{i}.T_Skm1_Sk(:,5:7)',sensorData{1}.T_Skm1_Sk(:,5:7)',weight);
    estVec(i,:) = R2V(temp);
end

end

