function [ prob ] = SystemProbR( sensorData, estVec )
%SYSTEMPROBR uses variance to find a measure of the systems probablity of
%   being correct (lower score == better)
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   sensorData- nx1 cell containing sensor data sturcts
%   estVec- (n-1)x3 matrix of rotations for each sensor (1st sensor info
%       not given as it will just be 0,0,0)
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

%set first element to zeros
estVec = [0,0,0;estVec];

%convert vectors to rotation matricies
estMat = cell(length(sensorData),1);
estMat{1} = eye(3);
for i = 2:length(sensorData)
    estMat{i} = V2R(estVec(i,:));
end

%find probablity of system
prob = 0;
for a = 1:length(sensorData)
    for b = 1:length(sensorData)
        %ensure no repeats
        if(a < b)
            %get rotation and variance
            Rab = (estMat{a}'*estMat{b})';
            VA = sensorData{a}.T_Var_Skm1_Sk(:,4:6)';
            VB = sensorData{b}.T_Var_Skm1_Sk(:,4:6)';
            
            estA = sensorData{a}.T_Skm1_Sk(:,4:6)';
            estB = sensorData{b}.T_Skm1_Sk(:,4:6)';
            
            %find position error
            err = Rab*estA - estB;
            
            %find weighted error
            %temp = cprobR(err, VA, VB, Rab);
            %temp = sum(temp);
            %temp = sum(sqrt(temp));
            %temp = sqrt(sum(temp(:)));
            
            temp = -logpdf(err,VA,VB,Rab);
            
            %add error
            prob = prob + temp;
        end
    end
end

end

