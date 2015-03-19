function [ sensorData ] = SolveScale4( sensorData, rotVec, rotVar )


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
            err = zeros(3,100);
            for k = 1:100
                err(:,k) = R2V(V2R(rotVec(a,:) + randn(1,3).*sqrt(rotVar(a,:)))\V2R(rotVec(b,:) + randn(1,3).*sqrt(rotVar(b,:))));
            end
            rVec{a,b} = mean(err,2)';
            rVar{a,b} = var(err,[],2)';
        end
    end
end

scale = zeros(size(sensorData{1}.T_Skm1_Sk,1),1);
    
for i = 1:size(sensorData{1}.T_Skm1_Sk,1)
    R = rVec{1,2};
    vR = rVar{1,2};

    A = sensorData{1}.T_Skm1_Sk(i,:);
    vA = sensorData{1}.T_Var_Skm1_Sk(i,:);
    B = sensorData{2}.T_Skm1_Sk(i,:);
    vB = sensorData{2}.T_Var_Skm1_Sk(i,:);

    scale(i) = fminsearch(@(S) OptS(S,R,vR,A,vA,B,vB),A(1,4));
    
   i 
end

sensorData{2}.T_Skm1_Sk(:,4) = scale;

end

function [err] = OptS(S,R,vR,A,vA,B,vB)
    
    [tA,vtA] = ts2t(A(:,1:4), vA(:,1:4));
    [tB,vtB] = ts2t([B(:,1:3),S], [vB(:,1:3),0]);

    RA = A(:,5:7);
    vRA = vA(:,5:7);
    RB = B(:,5:7);
    vRB = vB(:,5:7);

    offset = 0.0001;
    
    eqA = @(R,RA,RB,tA,tB) V2R(R)'*(eye(3)-V2R(RB))*(V2R(R)*tB'-tA');
    eqB = @(R,RA,RB,tA,tB) (V2R(RA)-eye(3))*(V2R(R)'*tA'-tB');
    
    [xA, vXA] = IndVar(offset, eqA, R,vR,RA,vRA,RB,vRB,tA,vtA,tB,vtB);
    [xB, vXB] = IndVar(offset, eqB, R,vR,RA,vRA,RB,vRB,tA,vtA,tB,vtB);
                
    err = (xA-xB).^2;
    err = err./(vXA+vXB);
    err = sum(err(:));

end

