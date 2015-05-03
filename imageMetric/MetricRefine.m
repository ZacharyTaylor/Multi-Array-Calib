function [ TGridR, vTGridR ] = MetricRefine( TGrid, vTGrid, sensorData, numScans )
%METRICREFINE Summary of this function goes here
%   Detailed explanation goes here

TGridR = cell(length(sensorData));
vTGridR = cell(length(sensorData));

for i = 1:size(TGrid,1);
    for j = 1:size(vTGrid,1);
        if(i<j)
            %check that sensors overlap
            overlap = CalcSensorOverlap(TGrid{i,j}, sensorData{i}, sensorData{j});
            if(overlap < 0.1)
                [TGridR{i,j},vTGridR{i,j}] = IndVar(0.01,@V2S,TGrid{i,j},vTGrid{i,j});
                continue;
            end
            
            %perform velodyne-camera matching
            if(and(strcmpi(sensorData{i}.type,'velodyne'),strcmpi(sensorData{j}.type,'camera')))
                [TGridR{i,j}, vTGridR{i,j}] = RefineVelCam(TGrid{i,j},vTGrid{i,j},sensorData{i},sensorData{j},'Colour',numScans);
            end
            
            %perform camera-camera matching
            if(and(strcmpi(sensorData{i}.type,'camera'),strcmpi(sensorData{j}.type,'camera')))
                [TGridR{i,j}, vTGridR{i,j}] = RefineCamCam(sensorData{i},sensorData{j},numScans);
            end

            %combine inputs using scale representation to handle cameras
            if(~isempty(TGridR{i,j}))

                [A,VA] = IndVar(0.01,@V2S,TGridR{i,j},vTGridR{i,j});
                [B,VB] = IndVar(0.01,@V2S,TGrid{i,j},vTGrid{i,j});
                    
                %cameras don't have scale
                if(and(strcmpi(sensorData{i}.type,'camera'),strcmpi(sensorData{j}.type,'camera')))
                    VA(4) = inf;
                end
                
                A(~isfinite(VA)) = B(~isfinite(VA));
                VA(~isfinite(VA)) = VB(~isfinite(VA));

                TGridR{i,j} = A;
                vTGridR{i,j} = VA;
                %[TGridR{i,j}, vTGridR{i,j}] = CombEst(A,VA,B,VB);
                
            end
        end
    end
end


end

