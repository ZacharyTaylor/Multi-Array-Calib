function outVec = OptR( sensorData, estVec )
%OPTR Optimize rotation based on inital guess
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   sensorData- nx1 cell containing sensor data sturcts
%   estVec- nx3 matrix of rotations for each sensor
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   outVec- nx3 matrix of the rotation for each sensor
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
outVec = fminsearch(@(estVec) SystemProbR(sensorData, estVec),estVec(2:end,:), options);
outVec = [0,0,0;outVec];

end

