function [ sensorData ] = EasyScale( sensorData, rotVec, rotVar, tranVec, tranVar )

%divide into camera and non-camera sensors
nonCamData = {};
nonCamIdx = {};
camData = {};
camIdx = {};

for i = 1:length(sensorData)
    if(strcmp(sensorData{i}.type,'camera'))
        camData = [camData,sensorData{i}];
        camIdx = [camIdx,i];
    else
        nonCamData = [nonCamData,sensorData{i}];
        nonCamIdx = [nonCamIdx,i];
    end
end

%if all cameras ext
if(isempty(nonCamIdx))
    return;
end

for i = 1:length(camData)
    
    %get joint rotation
    R = zeros(length(nonCamData),3);
    vR = zeros(length(nonCamData),3);
    for k = 1:length(nonCamData)
        output = @(r1,r2) R2V(V2R(r1)*V2R(r2))';
        [R(k,:),vR(k,:)] = IndVar(0.001, output, rotVec(camIdx{i},:),rotVar(camIdx{i},:),rotVec(nonCamIdx{k},:),rotVar(nonCamIdx{k},:));
    end
    
    tA = nonCamData{1}.T_Skm1_Sk(:,1:3);
    vtA = nonCamData{1}.T_Var_Skm1_Sk(:,1:3);
    tB = camData{i}.T_Skm1_Sk(:,1:3);
    vtB = camData{i}.T_Var_Skm1_Sk(:,1:3);

    RA = nonCamData{1}.T_Skm1_Sk(:,4:6);
    vRA = nonCamData{1}.T_Var_Skm1_Sk(:,4:6);
    RB = camData{i}.T_Skm1_Sk(:,4:6);
    vRB = camData{i}.T_Var_Skm1_Sk(:,4:6);
       
    for j = 1:size(camData{i}.T_Skm1_Sk,1)
        %estimate scale
        [s,sV] = IndVar(0.001,@findScale,R(1,:),vR(1,:),tranVec(2,:),tranVar(2,:),RA(j,:),vRA(j,:),tA(j,:),vtA(j,:),RB(j,:),vRB(j,:),tB(j,:),vtB(j,:));
        
        %form single estimate
        sV = 1./sV;
        s = sum(s.*sV);
        sV = 1./sum(sV);
        s = s.*sV;
        
        sensorData{camIdx{i}}.T_Var_Skm1_Sk(j,1:3) = sV.*(tB(j,:).^2) + (s.^2).*vtB(j,:);
        sensorData{camIdx{i}}.T_Skm1_Sk(j,1:3) = s.*tB(j,:);
    end
end

warning('on','MATLAB:nearlySingularMatrix');

end

function [scale] = findScale(R,t,RA,tA,RB,tB)

    scale = zeros(6,1);
    scale(1:3) = (V2R(R)'*((V2R(RA)-eye(3))*t' + tA'))./tB';
    scale(4:6) = (V2R(R)'*tA' - (eye(3)-V2R(RB))*V2R(R)'*t')./tB';
end