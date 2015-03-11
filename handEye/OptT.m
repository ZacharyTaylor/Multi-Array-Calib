function [ tranVec ] = OptT( sensorData, estVec, rotVec, rotVar )
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

%refine translation estimate and record result
options = optimset('MaxFunEvals',100000,'MaxIter',5000);
estVec = estVec(2:end,1:3);
tranVec = fminsearch(@(estVec) SystemProbT( sensorData, estVec, rVec, rVar),estVec, options);
tranVec = [0,0,0;tranVec];

end

