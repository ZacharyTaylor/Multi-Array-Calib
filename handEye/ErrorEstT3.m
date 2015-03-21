function [ tranVar ] = ErrorEstT3( sensorData, tranVec, rotVec, rotVar )
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

%convert rot vector to rotmats
rotMat = zeros(3,3,size(sensorData,1));
for i = 1:size(rotMat,3)
    rotMat(:,:,i) = V2R(rotVec(i,:));
end

%combine rotation estimations
rVec = cell(size(rotVec,1));
rVar = rVec;
for a = 1:size(rVec,1)
    for b = 1:size(rVec,1)
        if(a <= b)
            output = @(r1,r2) R2V(V2R(r1)*V2R(r2))';
            [rVec{a,b},rVar{a,b}] = IndVar(0.001, output, rotVec(a,:),rotVar(a,:),rotVec(b,:),rotVar(b,:));
        end
    end
end

a = 1;
b = 2;

R = rVec{a,b};
vR = rVar{a,b};

t = V2R(rVec{1,a})'*(tranVec(b,:) - tranVec(a,:))';

[tA,vtA] = ts2t(sensorData{a}.T_Skm1_Sk(:,1:4), sensorData{1}.T_Var_Skm1_Sk(:,1:4));
[tB,vtB] = ts2t(sensorData{b}.T_Skm1_Sk(:,1:4), sensorData{b}.T_Var_Skm1_Sk(:,1:4));

RA = sensorData{a}.T_Skm1_Sk(:,5:7);
vRA = sensorData{a}.T_Var_Skm1_Sk(:,5:7);
RB = sensorData{b}.T_Skm1_Sk(:,5:7);
vRB = sensorData{b}.T_Var_Skm1_Sk(:,5:7);

runFunc = @(R,vR,tA,vtA,RA,vRA,tB,vtB,RB,vRB) findTran(R,vR,tA,vtA,RA,vRA,tB,vtB,RB,vRB,t);

[~,tranVar] = IndVarVec(0.01, runFunc,R,vR,tA,vtA,RA,vRA,tB,vtB,RB,vRB);

end

function [t] = findTran(R,vR,tA,vtA,RA,vRA,tB,vtB,RB,vRB,t)

t = fminsearch(@(t) -logpdfTS(R',vR',tA',vtA',RA',vRA',tB',vtB',RB',vRB',t'),t);

end
