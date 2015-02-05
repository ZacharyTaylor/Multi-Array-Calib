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
    sensorInt.T_Var_S1_Sk = zeros(length(times),6);
    sensorInt.T_Var_Skm1_Sk = zeros(length(times),6);
    sensorInt.time = times(:);
    if(length(sensorInt.files) > 1)
        sensorInt.files = repmat(sensorInt.files(1),length(times),1);
    end
            
%     %for each time point
%     for i = 1:length(times)
% 
%         %find closest matching time
%         tdiff = double(times(i)) - double(sensorData.time);
% 
%         %get matching index
%         [~,idx] = min(abs(tdiff));
%         tdiff = tdiff(idx);
% 
%         %get base transform
%         tform = V2T(sensorData.T_S1_Sk(idx,:));
%         
%         %get base variance
%         sensorInt.T_Var_S1_Sk(i,:) = sensorData.T_Var_S1_Sk(idx,:);
% 
%         %get closest file
%         if(size(sensorData.files,1) > 1)
%             sensorInt.files(i) = sensorData.files(idx);
%         end
% 
%         %get fractional transform
%         if(idx < length(sensorData.time))
%             %get closest relative tform
%             if(tdiff < 0)
%                 tformF = V2T(sensorData.T_Skm1_Sk(idx,:));
%                 varF = sensorData.T_Var_Skm1_Sk(idx,:);
%                 
%                 %get time difference
%                 tratio = tdiff/(double(sensorData.time(idx))-double(sensorData.time(idx-1)));
%             
%             else
%                 tformF = V2T(sensorData.T_Skm1_Sk(idx+1,:));
%                 varF = sensorData.T_Var_Skm1_Sk(idx+1,:);
%                 
%                 %get time difference
%                 tratio = tdiff/(double(sensorData.time(idx+1))-double(sensorData.time(idx)));
%             end
% 
%             %interpolate
%             varF = tratio.*varF;
%             tformF = TformInterp(tformF,tratio);
% 
%             %add fractional component back to tform
%             tform = tform*tformF;
%             sensorInt.T_Var_S1_Sk(i,:) = sensorInt.T_Var_S1_Sk(i,:) + varF;
%         end
% 
%         sensorInt.T_S1_Sk(i,:) = T2V(tform);
%     end

    sensorInt.T_Var_S1_Sk = interp1(double(sensorData.time), sensorData.T_Var_S1_Sk, double(sensorInt.time),'pchip');
    sensorInt.T_S1_Sk = interp1(double(sensorData.time), sensorData.T_S1_Sk, double(sensorInt.time),'pchip');
        
    %split into relative transforms
    for i = 2:size(sensorInt.T_S1_Sk,1)
        sensorInt.T_Skm1_Sk(i,:) = T2V(V2T(sensorInt.T_S1_Sk(i-1,:))\V2T(sensorInt.T_S1_Sk(i,:)));
    end

    %split up variance
    for i = size(sensorInt.T_Var_Skm1_Sk,1):-1:2
        sensorInt.T_Var_Skm1_Sk(i,:) = sensorInt.T_Var_S1_Sk(i,:) - sensorInt.T_Var_S1_Sk(i-1,:);
    end
    
    if(strcmp(sensorInt.type,'nav'))
        sensorInt.T_Var_Skm1_Sk = repmat(median(sensorInt.T_Var_Skm1_Sk),size(sensorInt.T_Var_Skm1_Sk,1),1);
    end

end
    