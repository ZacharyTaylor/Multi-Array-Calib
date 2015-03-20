function [ sensorData ] = EasyScale( sensorData, rotVec, rotVar, tranVec )

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

for i = 1:length(camData)
    
    scaleA = ones(size(camData{i}.T_Skm1_Sk,1),3);
    scaleB = ones(size(camData{i}.T_Skm1_Sk,1),3);
    
    %get joint rotation
    samples = 100;
    R = zeros(3,length(nonCamData),samples);
    for k = 1:length(nonCamData)
        for j = 1:100
            R(:,k,j) = R2V(V2R(rotVec(camIdx{i},:)+randn(1,3).*rotVar(camIdx{i},:))/V2R(rotVec(nonCamIdx{k},:)+randn(1,3).*rotVar(nonCamIdx{k},:)));
        end
    end
    
    VR = var(R,[],3)';
    R = mean(R,3)';
    
    [tA,vtA] = ts2t(nonCamData{1}.T_Skm1_Sk(:,1:4), nonCamData{1}.T_Var_Skm1_Sk(:,1:4));
    [tB,vtB] = ts2t([camData{i}.T_Skm1_Sk(:,1:3),ones(size(camData{i}.T_Skm1_Sk,1),1)], [camData{i}.T_Var_Skm1_Sk(:,1:3),zeros(size(camData{i}.T_Skm1_Sk,1),1)]);

    RA = nonCamData{1}.T_Skm1_Sk(:,5:7);
    vRA = nonCamData{1}.T_Var_Skm1_Sk(:,5:7);
    RB = camData{i}.T_Skm1_Sk(:,5:7);
    vRB = camData{i}.T_Var_Skm1_Sk(:,5:7);
       
    for j = 1:size(camData{i}.T_Skm1_Sk,1)
        scaleA(j,:) = (V2R(R)'*((V2R(RA(j,:))-eye(3))*tranVec(2,:)' + tA(j,:)'))./tB(j,:)';
        scaleB(j,:) = (V2R(R)'*tA(j,:)' - (eye(3)-V2R(RB(j,:)))*V2R(R)'*tranVec(2,:)')./tB(j,:)';
         
    end
    
    sensorData{camIdx{i}}.T_Var_Skm1_Sk(:,4) = 0;
    sensorData{camIdx{i}}.T_Skm1_Sk(:,4) = (scaleA(:,3) + scaleB(:,3))/2;

end

warning('on','MATLAB:nearlySingularMatrix');

end