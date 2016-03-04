function [ sensorData ] = IntSensorData( sensorData, times )
%INTSENSORDATA interpolates sensor data at set times
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   sensorData- either a nx1 cell containing sensor data sturcts
%       or a single sensor data struct
%   times- mx1 vector of times to interpolate at, must be increasing
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
validateattributes(times,{'numeric'},{'positive','vector','increasing'});

if(iscell(sensorData))
    for i=1:length(sensorData)
        sensorData{i} = IntData(sensorData{i}, times);
    end
elseif(isstruct(sensorData))
    sensorData = IntData(sensorData, times);
else
    error('sensorData must be a struct of cell of structs');
end

end

function[ sensorInt ] = IntData( sensorData, times )
    %setup ouput
    sensorInt = sensorData;
    sensorInt.T_S1_Sk = zeros(length(times),6);
    sensorInt.T_Skm1_Sk = zeros(length(times),6);
    sensorInt.T_Skm1_Sk_raw = zeros(length(times),6);
    sensorInt.T_Var_S1_Sk = zeros(length(times),6);
    sensorInt.T_Var_Skm1_Sk = zeros(length(times),6);
    sensorInt.times = times(:);

    sensorInt.T_Var_S1_Sk = interp1(double(sensorData.times), sensorData.T_Var_S1_Sk, double(sensorInt.times),'pchip');
    sensorInt.T_S1_Sk = interp1(double(sensorData.times), sensorData.T_S1_Sk, double(sensorInt.times),'pchip');
        
    %split into relative transforms
    for i = 2:size(sensorInt.T_S1_Sk,1)
        sensorInt.T_Skm1_Sk(i,:) = T2V(V2T(sensorInt.T_S1_Sk(i-1,:))\V2T(sensorInt.T_S1_Sk(i,:)));
    end

    %split up variance
    if(strcmpi(sensorInt.type,'vicon'))
        sensorInt.T_Var_Skm1_Sk = sensorInt.T_Var_S1_Sk;
    else
        for i = size(sensorInt.T_Var_Skm1_Sk,1):-1:2
            sensorInt.T_Var_Skm1_Sk(i,:) = sensorInt.T_Var_S1_Sk(i,:) - sensorInt.T_Var_S1_Sk(i-1,:);
        end
    end
end
    