function [ varVec ] = ErrorEstCR( sensorData, rotVec, step )
%ERRORESTR estimate cramer rao lower bound for error variance
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   sensorData- nx1 cell containing sensor data sturcts
%   estVec- nx3 matrix of rotations for each sensor
%   step- step between test points for numercial differentiation
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
validateattributes(rotVec,{'numeric'},{'size',[length(sensorData),3]});
validateattributes(step,{'numeric'},{'scalar','positive','nonzero'});

%pull usful info out of sensorData
RData = zeros(size(sensorData{1}.T_Skm1_Sk,1),3,length(sensorData));
vRData = RData;

for i = 1:length(sensorData)
    RData(:,:,i) = sensorData{i}.T_Skm1_Sk(:,4:6);
    vRData(:,:,i) = sensorData{i}.T_Var_Skm1_Sk(:,4:6);
end

rotVec = rotVec(2:end,:);
varVec = zeros(size(rotVec));

for i = 1:length(rotVec(:))
    out = zeros(3,1);
    for j = 1:3
        temp = rotVec;
        temp(i) = temp(i) + step*(j-2);
        out(j) = SystemProbR(RData, vRData, temp);
    end
    varVec(i) = (step^2)./diff(out,2);
end

varVec = [0,0,0;varVec];

end

