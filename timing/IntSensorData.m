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

function[ SensorInt ] = IntData( sensorData, times )
    %setup ouput
    SensorInt = sensorData;
    SensorInt.T_S1_Sk = zeros(1:length(times),1:6);
    SensorInt.T_Skm1_Sk = zeros(1:length(times),1:6);
    SensorInt.T_Var_S1_Sk = zeros(1:length(times),1:6);
    SensorInt.T_Var_Skm1_Sk = zeros(1:length(times),1:6);
    SensorInt.times = times;
    if(length(SensorInt.files) > 1)
        SensorInt.files = SensorInt.files(1:length(times),1);
    end

    %for each time point
    for i = 1:size(times(:),1)

        %find closest matching time
        tdiff = double(sensorData.time) - double(times(i));

        %get matching index
        [~,idx] = min(abs(tdiff));
        tdiff = tdiff(idx);

        %get base transform
        tform = V2T(sensorData.T_S1_Sk(idx,:)');

        %get closest file
        if(size(sensorData.files,1) > 1)
            SensorInt.files(i) = sensorData.files(idx);
        end

        %get variance
        SensorInt.T_Var_S1_Sk(i,:) = SensorData.T_Var_S1_Sk(idx,:);

        %get fractional transform
        if(idx > 1)
            %get closest relative tform
            if(tdiff > 0)
                tformF = V2T(sensorData.T_Skm1_Sk(idx,:)');
            else
                tformF = V2T(sensorData.T_Skm1_Sk(idx-1,:)');
            end

            %get time difference
            tratio = tdiff/(double(sensorData.time(idx))-double(sensorData.time(idx-1)));

            %interpolate
            tformF = TformInterp(tformF,-tratio);

            %add fractional component back to tform
            tform = tform*tformF;
        end

        SensorInt.T_S1_Sk(i,:) = T2V(tform);
    end

    %split into relative transforms
    for i = 2:size(SensorInt.T_S1_Sk,1)
        SensorInt.T_Skm1_Sk(i,:) = T2V(vec2tran(SensorInt.T_S1_Sk(i-1,:)')\vec2tran(SensorInt.T_S1_Sk(i,:)'))';
    end

    %split up variance
    for i = size(SensorInt.T_Var_Skm1_Sk,1):-1:2
        SensorInt.T_Var_Skm1_Sk(i,:) = SensorInt.T_Var_S1_Sk(i,:) - SensorInt.T_Var_S1_Sk(i-1,:);
    end

end
    