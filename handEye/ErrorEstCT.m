function [ varVec ] = ErrorEstCT( sensorData, tranVec, rotVec, rotVar, step )
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

for i = 1:length(tranVec(:))
    out = zeros(3,1);
    for j = 1:3
        temp = tranVec;
        temp(i) = temp(i) + step*(j-2);
        out(j) = SystemProbT(TData, vTData, s, temp, rotVec, rotVar);
    end
    varVec(i) = (step^2)./diff(out,2);
end

varVec = [0,0,0;varVec];

end

