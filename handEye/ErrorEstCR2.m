function [ varVec ] = ErrorEstCR( sensorData, estVec, step )
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
validateattributes(estVec,{'numeric'},{'size',[length(sensorData),3]});
validateattributes(step,{'numeric'},{'scalar','positive','nonzero'});

%convert vectors to rotation matricies
estMat = cell(length(sensorData),1);
estMat{1} = eye(3);
for i = 2:length(sensorData)
    estMat{i} = V2R(estVec(i,:));
end

varVec = zeros(length(sensorData),3);

p = zeros(6,1);
for b = 2:length(sensorData)
    Rab = estMat{b}*estMat{1}';
    Rab = R2V(Rab);

    estA = sensorData{1}.T_Skm1_Sk(:,5:7)';
    estB = sensorData{b}.T_Skm1_Sk(:,5:7)';
    
    varA = sensorData{1}.T_Var_Skm1_Sk(:,5:7)';
    varB = sensorData{b}.T_Var_Skm1_Sk(:,5:7)';

    err = V2R(Rab)*estA - estB;
    m = logpdf(err,varA,varB,R);
    
    for c = 1:3
        R = Rab;
        R(c) = R(c) + step*(d-2);
        R = V2R(R');
        err = R*estA - estB;

        p(d,c) = logpdf(err,varA,varB,R);
    end
    varVec(b,:) = -(step^2)./diff(p,2);
end

end

