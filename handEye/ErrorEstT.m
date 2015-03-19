function [ outVec ] = ErrorEstT( sensorData, estVec, rotVec, rotVar, samples )

%check inputs
validateattributes(sensorData,{'cell'},{'vector'});
for i = 1:length(sensorData)
    validateattributes(sensorData{i},{'struct'},{});
end
validateattributes(estVec,{'numeric'},{'size',[length(sensorData),3]});
validateattributes(rotVec,{'numeric'},{'size',[length(sensorData),3]});

%convert rot vector to rotmats
rotMat = zeros(3,3,size(sensorData,1));
for i = 1:size(rotMat,3)
    rotMat(:,:,i) = V2R(rotVec(i,:));
end

%refine translation estimate and record result
options = optimset('MaxFunEvals',100000,'MaxIter',5000);

outVec = zeros([size(estVec),samples]);
for i = 1:samples
    %bootstrap data
    idx = datasample(1:size(sensorData{1}.time,1),size(sensorData{1}.time,1));
    sData = SensorDataSubset(sensorData, idx);
    
    r = rotVec;
    r(2:end,:) = fminsearch(@(rotVec) SystemProbR(sensorData, rotVec),rotVec(2:end,:), options);
    
    %combine rotation estimations
    rVec = cell(size(rotVec,1));
    rVar = rVec;
    for a = 1:size(rVec,1)
        for b = 1:size(rVec,1)
            if(a <= b)
                temp = zeros(3,100);
                for k = 1:100
                    temp(:,k) = R2V(V2R(r(a,:) + randn(1,3).*sqrt(rotVar(a,:)))\V2R(r(b,:) + randn(1,3).*sqrt(rotVar(b,:))));
                end
                rVec{a,b} = mean(temp,2)';
                rVar{a,b} = var(temp,[],2)';
            end
        end
    end
    
    outVec(2:end,1:3,i) = fminsearch(@(estVec) SystemProbT( sData, estVec, rVec, rVar),estVec(2:end,1:3), options);
end

outVec = var(outVec,[],3);

end

