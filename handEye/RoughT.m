function [ estVecT ] = RoughT( sensorData, rotVec )
%ROUGHT Uses least squares approach to find translation estimate
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   sensorData- nx1 cell containing sensor data sturcts
%   rotVec- nx3 matrix of the rotation for each sensor
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   estVecT- nx3 matrix of the translation for each sensor
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

%convert rot vector to rotmats
RMat = zeros(3,3,size(sensorData,1));
for j = 1:size(RMat,3)
    RMat(:,:,j) = V2R(rotVec(j,:));
end

%record number of transforms for quick reference
s = size(sensorData{1}.T_Skm1_Sk,1);

%preallocate memory
estVecT = zeros(size(sensorData,1),3);

%for each sensor
for b = 2:size(sensorData,1)
    
    %find the max variance and convert to weighting
    weight = 1./sqrt(max(sensorData{1}.T_Var_Skm1_Sk(:,1:3),[],2) + max(sensorData{b}.T_Var_Skm1_Sk(:,1:3),[],2));
    weight = weight(:);

    Rb = zeros(3*s,3);
    ta = zeros(3*s,1);

    for j = 1:s
        Rb(3*j-2:3*j,1:3) = V2R(sensorData{b}.T_Skm1_Sk(j,4:6)) - eye(3);
        Rb(3*j-2:3*j,:) = weight(j).* Rb(3*j-2:3*j,:);

        ta(3*j-2:3*j) = RMat(:,:,b)*sensorData{1}.T_Skm1_Sk(j,1:3)' - sensorData{b}.T_Skm1_Sk(j,1:3)';
        ta(3*j-2:3*j) = weight(j).* ta(3*j-2:3*j);
    end

    temp = (Rb\ta);
    estVecT(b,:) = temp(1:3)';
end


end

