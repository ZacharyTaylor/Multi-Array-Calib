function [ sensorData ] = SolveScale( sensorData, rotVec, rotVar )

%divide into camera and non-camera sensors
nonCamData = {};
nonCamIdx = {};
camData = {};
camIdx = {};

warning('off','MATLAB:nearlySingularMatrix');

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
    
    s = ones(size(camData{i}.T_Skm1_Sk,1),1);
    sV = s;
    
    for j = 3:size(camData{i}.T_Skm1_Sk,1)
        VA12 = nonCamData{1}.T_Var_Skm1_Sk(j-1,:);
        A12 = nonCamData{1}.T_Skm1_Sk(j-1,:);

        VB12 = camData{i}.T_Var_Skm1_Sk(j-1,:) - [0,0,0,camData{i}.T_Var_Skm1_Sk(j-2,4),0,0,0];
        B12 = camData{i}.T_Skm1_Sk(j-1,:);

        VA23 = nonCamData{1}.T_Var_Skm1_Sk(j,:);
        A23 = nonCamData{1}.T_Skm1_Sk(j,:);

        VB23 = camData{i}.T_Var_Skm1_Sk(j,:) - [0,0,0,camData{i}.T_Var_Skm1_Sk(j-1,4),0,0,0];
        B23 = camData{i}.T_Skm1_Sk(j,:);

        R = V2R(rotVec(camIdx{i},:))/V2R(rotVec(nonCamIdx{i},:));
        VR = abs(CrossMat(rotVec(camIdx{i},:)) + CrossMat(rotVec(nonCamIdx{i},:)));
        
        [eq1, eq1V] = TEqVar(A23, VA23, B23, VB23, R, VR);
        [eq2, eq2V] = TEqVar(A12, VA12, B12, VB12, R, VR);
        [eq3, eq3V] = TEqVar2(B23, VB23);
        [eq4, eq4V] = TEqVar2(B12, VB12);

        N = eq1-eq2;
        NV = eq1V + eq2V;
        
        D = eq3-eq4;
        invDV = (eq3V+eq4V)./(D.^4);
        
        s(j-1) = D\N;
        %sV(j-1) = (1./D).*NV + N.*invDV;
    end
    
    sensorData{camIdx{i}}.T_Skm1_Sk(:,4) = s.*sensorData{camIdx{i}}.T_Skm1_Sk(:,4);
end

warning('on','MATLAB:nearlySingularMatrix');

end