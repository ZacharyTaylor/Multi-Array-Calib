function [ prob ] = SystemProbT( sensorData, estVec, rotVec, rotVar )
%SYSTEMPROBT estimate error from variance in trans vectors after alignment
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   sensorData- nx1 cell containing sensor data sturcts
%   tformMat- nx1 cell of tform mats in a vector format (see OptT)
%   estVec- (n-1)x3 matrix of translations for each sensor (1st sensor info
%       not given as it will just be 0,0,0)
%   rotMat- 3x3xn matrix of rotation matrix for each sensor (1st sensor
%       info not given as it will just be 0,0,0)
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   prob- measure of system likelihood (note not an actual probabilty)
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
validateattributes(estVec,{'numeric'},{'size',[length(sensorData)-1,3]});
%validateattributes(rotVec,{'numeric'},{'size',[length(sensorData),3]});

%set first element to zeros
estVec = [0,0,0;estVec];

%find probablity of system
prob = 0;
for a = 1:length(sensorData)
    for b = 1:length(sensorData)
        if(a < b)
            R = rotVec{a,b};
            vR = rotVar{a,b};
            
            t = V2R(rotVec{1,a})'*(estVec(b,:) - estVec(a,:))';
            
            [tA,vtA] = ts2t(sensorData{a}.T_Skm1_Sk(:,1:4), sensorData{1}.T_Var_Skm1_Sk(:,1:4));
            [tB,vtB] = ts2t(sensorData{b}.T_Skm1_Sk(:,1:4), sensorData{b}.T_Var_Skm1_Sk(:,1:4));
            tA = tA'; vtA = vtA';
            tB = tB'; vtB = vtB';
            
            RB = sensorData{b}.T_Skm1_Sk(:,5:7)';
            vRB = sensorData{b}.T_Var_Skm1_Sk(:,5:7)';
            
            temp = -logpdfT(R,vR,tA,vtA,tB,vtB,RB,vRB,t);
            
            prob = prob + temp;
        end
    end
end

end

