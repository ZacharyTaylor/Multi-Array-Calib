function [ varVec ] = ErrorEstR( sensorData, estVec )
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

%convert vectors to rotation matricies
estMat = cell(length(sensorData),1);
estMat{1} = eye(3);
for i = 2:length(sensorData)
    estMat{i} = V2R(estVec(i,:));
end

varVec = zeros(length(sensorData),3);
for b = 2:length(sensorData)
    Rab = estMat{b}*estMat{1}';

    estA = sensorData{1}.T_Skm1_Sk(:,4:6)';
    estB = sensorData{b}.T_Skm1_Sk(:,4:6)';
    
    varA = sensorData{1}.T_Var_Skm1_Sk(:,4:6)';
    varB = sensorData{b}.T_Var_Skm1_Sk(:,4:6)';

    err = Rab*estA - estB;
    %err = diag(var(err'))/(estA*estA');
    err = (varA*varA')/(estA*estA');
    varVec(b,:) = diag(err);
end

end

