function [ outVec ] = ErrorEstWR( sensorData, estVec )
%ERRORESTR estimate error using highly conservative estamation
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

%add perfect match to sensor data
pM = [1,1,1];
distV = 0.0001

for i = 1:size(sensorData)
    tM = V2R(estVec(i,:))*pM';
    %pVar = mean(sensorData{i}.T_Var_Skm1_Sk,1);
    sensorData{i}.T_Skm1_Sk = [sensorData{i}.T_Skm1_Sk;[0,0,0,0,tM']];
    sensorData{i}.T_Var_Skm1_Sk = [sensorData{i}.T_Var_Skm1_Sk;distV*ones(1,7)];
end

dist = 0.01;
vD = zeros([size(estVec(2:end,:)),3]);
for i = 1:3
    sensorData{1}.T_Skm1_Sk(end,4+i) = sensorData{1}.T_Skm1_Sk(end,4+i) + dist;
    vD(:,:,i) = fminsearch(@(estVec) SystemProbR(sensorData, estVec),estVec(2:end,:), options);
    sensorData{1}.T_Skm1_Sk(end,4+i) = sensorData{1}.T_Skm1_Sk(end,4+i) - dist;
end


vD = vD - repmat(estVec(2:end,:),[1,1,size(vD,3)]);
vD = ((vD/dist).^2)/distV;
outVec = sum(vD,3);

outVec = [0,0,0;outVec];

