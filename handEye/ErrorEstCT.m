function [ varVec ] = ErrorEstCT( sensorData, estVec, rotVec, rotVar, step )
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
validateattributes(rotVec,{'numeric'},{'size',[length(sensorData),3]});

%convert rot vector to rotmats
rotMat = zeros(3,3,size(sensorData,1));
for i = 1:size(rotMat,3)
    rotMat(:,:,i) = V2R(rotVec(i,:));
end

%combine rotation estimations
rVec = cell(size(rotVec,1));
rVar = rVec;
for i = 1:size(rVec,1)
    for j = 1:size(rVec,1)
        if(i < j)
            temp = zeros(3,100);
            for k = 1:100
                temp(:,k) = R2V(V2R(rotVec(j,:) + randn(1,3).*sqrt(rotVar(j,:)))/V2R(rotVec(i,:) + randn(1,3).*sqrt(rotVar(i,:))));
            end
            rVec{i,j} = mean(temp,2)';
            rVar{i,j} = var(temp,[],2)';
        end
    end
end

varVec = zeros(length(sensorData),3);
p = zeros(3,3);
for b = 2:length(sensorData)
    R = rVec{1,b};
    vR = rVar{1,b};

    vtA = (repmat(sensorData{1}.T_Skm1_Sk(:,4),1,3).*abs(sensorData{1}.T_Var_Skm1_Sk(:,1:3)) + repmat(sensorData{1}.T_Var_Skm1_Sk(:,4),1,3).*abs(sensorData{1}.T_Skm1_Sk(:,1:3)))';
    vtB = (repmat(sensorData{b}.T_Skm1_Sk(:,4),1,3).*abs(sensorData{b}.T_Var_Skm1_Sk(:,1:3)) + repmat(sensorData{b}.T_Var_Skm1_Sk(:,4),1,3).*abs(sensorData{b}.T_Skm1_Sk(:,1:3)))';

    tA = (repmat(sensorData{1}.T_Skm1_Sk(:,4),1,3).*sensorData{1}.T_Skm1_Sk(:,1:3))';
    tB = (repmat(sensorData{b}.T_Skm1_Sk(:,4),1,3).*sensorData{b}.T_Skm1_Sk(:,1:3))';

    RB = sensorData{b}.T_Skm1_Sk(:,5:7)';
    vRB = sensorData{b}.T_Var_Skm1_Sk(:,5:7)';

    for c = 1:3
        for d = 1:3
            
            t = (estVec(b,:) - estVec(1,:))';
            t(c) = t(c) + step*(d-2);
            t = V2R(rVec{1,b})*t;
            
            p(d,c) = logpdfT(R,vR,tA,vtA,tB,vtB,RB,vRB,t);
        end
    end
    
    varVec(b,:) = -(step^2)./diff(p,2);
end

end

