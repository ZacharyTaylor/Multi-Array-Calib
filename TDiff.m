function [ sensorData ] = TDiff( sensorData, rotVec, rotVar )


%check inputs
validateattributes(sensorData,{'cell'},{'vector'});
for i = 1:length(sensorData)
    validateattributes(sensorData{i},{'struct'},{});
end
validateattributes(rotVec,{'numeric'},{'size',[length(sensorData),3]});

%convert rot vector to rotmats
rotMat = zeros(3,3,size(sensorData,1));
for i = 1:size(rotMat,3)
    rotMat(:,:,i) = V2R(rotVec(i,:));
end

%combine rotation estimations
rVec = cell(size(rotVec,1));
rVar = rVec;
for a = 1:size(rVec,1)
    for b = 1:size(rVec,1)
        if(a <= b)
            temp = zeros(3,100);
            for k = 1:100
                temp(:,k) = R2V(V2R(rotVec(a,:) + randn(1,3).*sqrt(rotVar(a,:)))\V2R(rotVec(b,:) + randn(1,3).*sqrt(rotVar(b,:))));
            end
            rVec{a,b} = mean(temp,2)';
            rVar{a,b} = var(temp,[],2)';
        end
    end
end

err =zeros(size(sensorData{1}.T_Skm1_Sk,1),1);

%find error for each sensor
for a = 1:length(sensorData)
    for b = 1:length(sensorData)
        if(a < b)
            R = rVec{a,b};
            vR = rVar{a,b};
                       
            [tA,vtA] = ts2t(sensorData{a}.T_Skm1_Sk(:,1:4), sensorData{1}.T_Var_Skm1_Sk(:,1:4));
            [tB,vtB] = ts2t(sensorData{b}.T_Skm1_Sk(:,1:4), sensorData{b}.T_Var_Skm1_Sk(:,1:4));
            
            RA = sensorData{a}.T_Skm1_Sk(:,5:7);
            vRA = sensorData{a}.T_Var_Skm1_Sk(:,5:7);
            RB = sensorData{b}.T_Skm1_Sk(:,5:7);
            vRB = sensorData{b}.T_Var_Skm1_Sk(:,5:7);
            
            A = zeros(size(RA,1),3);
            B = zeros(size(RA,1),3);

            vA = zeros(size(RA,1),3);
            vB = zeros(size(RA,1),3);
            offset = 0.0001;
            
            for i = 1:size(A,1)
                eqA = @(R,RA,RB,tA,tB) V2R(R)'*(eye(3)-V2R(RB))*(V2R(R)*tB'-tA');
                eqB = @(R,RA,RB,tA,tB) (V2R(RA)-eye(3))*(V2R(R)'*tA'-tB');
                
                [A(i,:), vA(i,:)] = IndVar(offset, eqA, R,vR,RA(i,:),vRA(i,:),RB(i,:),vRB(i,:),tA(i,:),vtA(i,:),tB(i,:),vtB(i,:));
                [B(i,:), vB(i,:)] = IndVar(offset, eqB, R,vR,RA(i,:),vRA(i,:),RB(i,:),vRB(i,:),tA(i,:),vtA(i,:),tB(i,:),vtB(i,:));
                    
            end
            
            temp = sqrt(sum((A-B).^2,2));
            
            err = err + temp;
        end
    end
end

%ditch worst half of data
[~,idx] = sort(err);
idx = idx(1:ceil(size(idx,1)/2));
valid = false(size(err));
valid(idx) = true;
valid = find(valid);

sensorData = SensorDataSubset(sensorData, valid);

end

