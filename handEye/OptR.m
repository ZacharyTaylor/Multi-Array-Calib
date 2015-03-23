function outVec = OptR( sensorData, estVec )
%OPTR Optimize rotation based on inital guess
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   sensorData- nx1 cell containing sensor data sturcts
%   estVec- nx3 matrix of rotations for each sensor
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   outVec- nx3 matrix of the rotation for each sensor
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

%pull usful info out of sensorData
RData = zeros(size(sensorData{1}.T_Skm1_Sk,1),3,length(sensorData));
vRData = RData;

for i = 1:length(sensorData)
    RData(:,:,i) = sensorData{i}.T_Skm1_Sk(:,4:6);
    vRData(:,:,i) = sensorData{i}.T_Var_Skm1_Sk(:,4:6);
end

%refine rotation estimate and record result
options = optimset('MaxFunEvals',100000,'MaxIter',5000);
outVec = fminsearch(@(estVec) SystemProbR(RData, vRData, estVec),estVec(2:end,:), options);
outVec = [0,0,0;outVec];

end

