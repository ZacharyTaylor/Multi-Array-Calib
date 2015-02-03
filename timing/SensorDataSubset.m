function [ sensorData ] = SensorDataSubset( sensorData, idx )
%SENSORDATASUBSET selects subset of sensorData structure
%--------------------------------------------------------------------------
%   Inputs:
%--------------------------------------------------------------------------
%   sensorData- either a nx1 cell containing sensor data sturcts
%       or a single sensor data struct
%   idx- index of data points in the structs to keep
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   sensorData- either a nx1 cell containing sensor data sturcts
%       or a single sensor data struct
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
validateattributes(idx,{'numeric'},{'integer','positive','nonzero','vector'});

if(iscell(sensorData))
    for i=1:length(sensorData)
        sensorData{i} = FindSubset(sensorData{i}, idx);
    end
elseif(isstruct(sensorData))
    sensorData = FindSubset(sensorData, idx);
else
    error('sensorData must be a struct of cell of structs');
end

end
    

function [ sData ] = FindSubset( sData, idx )

    if(length(sData.files) > 1)
        sData.files = sData.files(idx,:);
    end

    sData.time = sData.time(idx,:);
    sData.T_Skm1_Sk = sData.T_Skm1_Sk(idx,:);
    sData.T_S1_Sk = sData.T_S1_Sk(idx,:);
    sData.T_Var_Skm1_Sk = sData.T_Var_Skm1_Sk(idx,:);
    sData.T_Var_S1_Sk = sData.T_Var_S1_Sk(idx,:);

end

