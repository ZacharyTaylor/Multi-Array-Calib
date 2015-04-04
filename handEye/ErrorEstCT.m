function [ varVec ] = ErrorEstCT( sensorData, tranVec, rotVec, rotVar )
%OPTR Optimize translation based on inital guess
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   sensorData- nx1 cell containing sensor data sturcts
%   estVec- nx3 matrix of rotations for each sensor
%   rotVec- nx3 matrix of rotations for each sensor
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   outVec- nx3 matrix of the translation for each sensor
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
validateattributes(tranVec,{'numeric'},{'size',[length(sensorData),3]});
validateattributes(rotVec,{'numeric'},{'size',[length(sensorData),3]});

%pull usful info out of sensorData
TData = zeros(size(sensorData{1}.T_Skm1_Sk,1),6,length(sensorData));
vTData = TData;
s = zeros(length(sensorData),1);

for i = 1:length(sensorData)
    TData(:,:,i) = sensorData{i}.T_Skm1_Sk;
    vTData(:,:,i) = sensorData{i}.T_Var_Skm1_Sk;
    s(i) = strcmpi(sensorData{i}.type,'camera');
end

tranVec = tranVec(2:end,:);
varVec = zeros(size(tranVec));

steps = [1,0.1,0.01,0.001,0.0001,0.00001,0.000001,0.0000001];
for i = 1:length(tranVec(:))
    out = zeros(length(steps),3);
    for j = 1:3
        for k = 1:length(steps)
            temp = tranVec;
            temp(i) = temp(i) + steps(k)*(j-2);
            out(k,j) = SystemProbT(TData, vTData, s, temp, rotVec, rotVar, false);
        end
    end
    out(:,1) = min(out(:,1),out(:,3));
    out(:,3) = out(:,1);
    out = (steps.^2)'./diff(out,2,2);
    varVec(i) = max(out);
end

varVec = [0,0,0;varVec];

end

