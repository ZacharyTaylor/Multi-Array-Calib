function [ varVec ] = ErrorEstCR( sensorData, rotVec )
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

%pull usful info out of sensorData
RData = zeros(size(sensorData{1}.T_Skm1_Sk,1),3,length(sensorData));
vRData = RData;

for i = 1:length(sensorData)
    RData(:,:,i) = sensorData{i}.T_Skm1_Sk(:,4:6);
    vRData(:,:,i) = sensorData{i}.T_Var_Skm1_Sk(:,4:6);
end

rotVec = rotVec(2:end,:);
varVec = zeros(size(rotVec));

steps = [1,0.1,0.01,0.001,0.0001,0.00001,0.000001,0.0000001];
for i = 1:length(rotVec(:))
    out = zeros(length(steps),3);
    for j = 1:3
        for k = 1:length(steps)
            temp = rotVec;
            temp(i) = temp(i) + steps(k)*(j-2);
            out(k,j) = SystemProbR(RData, vRData, temp, false);
        end
    end
    out(:,1) = min(out(:,1),out(:,3));
    out(:,3) = out(:,1);
    out = (steps.^2)'./diff(out,2,2);
    varVec(i) = max(out);
end

varVec = [0,0,0;varVec];

end

