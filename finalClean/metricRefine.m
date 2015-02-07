function [ tGrid, vGrid ] = MetricRefine( tGrid, vGrid, sensorData, minOverlap, matchNum )
%METRICREFINE uses an appropriate metric to refine the results of a scan

gs = size(tGrid,1);

for i = 1:gs
    for j = 1:gs
        if(i >= j)
            continue;
        end
        
        %If one sensor is a nav sensor
        if(or(strcmpi('nav',sensorData{i}.type),strcmpi('nav',sensorData{j}.type)))
            %if a nav sensor cannot refine solution
            continue;
        %If both sensors are cameras
        elseif(and(strcmpi('camera',sensorData{i}.type),strcmpi('camera',sensorData{j}.type)))
            overlap = CalcSensorOverlap(tGrid{i,j}, sensorData{i}, sensorData{j}, false);
            if(overlap > minOverlap)
                [tGrid{i,j}, vGrid{i,j}] = multiImageMatch(sensorData{j}, sensorData{i}, matchNum);
            end
        %If the 1st sensor is a lidar and the 2nd a camera
        elseif(and(strcmpi('velodyne',sensorData{i}.type),strcmpi('camera',sensorData{j}.type)))
            overlap = CalcSensorOverlap(tGrid{i,j}, sensorData{i}, sensorData{j}, true);
            if(overlap > minOverlap)
                [ tGrid{i,j}, vGrid{i,j} ] = OptVals(tGrid{i,j},vGrid{i,j},sensorData{i},sensorData{j},false, matchNum);
            end
        %If the 1st sensor is a camera and the 2nd a lidar
        elseif(and(strcmpi('camera',sensorData{i}.type),strcmpi('velodyne',sensorData{j}.type)))
            overlap = CalcSensorOverlap(tGrid{i,j}, sensorData{j}, sensorData{i}, true);
            if(overlap > minOverlap)
                [ tGrid{i,j}, vGrid{i,j} ] = OptVals(tGrid{i,j}',vGrid{i,j}',sensorData{j},sensorData{i},true, matchNum);
            end
        end
    end
end


end

