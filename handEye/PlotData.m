function [] = PlotData( sensorData, rotVec, fig )
%PlotData plots sensor data for easy visualization
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   sensorData- nx1 cell containing sensor data sturcts
%
%--------------------------------------------------------------------------
%   Optional Inputs:
%--------------------------------------------------------------------------
%   fig- figure handle to plot to
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
if(nargin > 2)
    validateattributes(fig,{'Figure'},{});
else
    fig = figure;
end

points = cell(size(sensorData));
leg = 'legend(';

cmap = hsv(length(sensorData));

set(0, 'CurrentFigure', fig);
hold on;

%generate points
for i = 1:length(sensorData)
    R = V2R(rotVec(i,:));
    points{i} = zeros(size(sensorData{i}.T_Skm1_Sk,1),3);
    temp= eye(4);
    temp(1:3,1:3) = R;
    for j = 1:size(sensorData{i}.T_Skm1_Sk,1)
        temp = temp*V2T(sensorData{i}.T_Skm1_Sk(j,:));
        points{i}(j,:) = temp(1:3,4);
    end
    
    plot3(points{i}(:,1),points{i}(:,2),points{i}(:,3),'Color',cmap(i,:));
    leg =[leg, '''', 'Sensor ', num2str(i),' ', sensorData{i}.type, ''''];
    if(i == length(sensorData))
        leg = [leg ');'];
    else
        leg = [leg ','];
    end
    
end

axis equal;
eval(leg);
drawnow;

