function [ TGridR, vTGridR ] = MetricRefine( TGrid, vTGrid, sensorData, numScans )
%METRICREFINE Summary of this function goes here
%   Detailed explanation goes here

TGridR = cell(length(sensorData));
vTGridR = cell(length(sensorData));

for i = 1:size(TGrid,1);
    for j = 1:size(vTGrid,1);
        if(i<j)
            if(and(strcmpi(sensorData{i}.type,'velodyne'),strcmpi(sensorData{j}.type,'camera')))
                [TGridR{i,j}, vTGridR{i,j}] = RefineVelCam(TGrid{i,j},vTGrid{i,j},sensorData{i},sensorData{j},'Colour',numScans);
            end
        end
    end
end


end

