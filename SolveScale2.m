function [ sensorData ] = SolveScale2( sensorData, rotVec, rotVar )

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
    
    samples = 100;
    R = zeros(3,length(nonCamData),samples);
    for k = 1:length(nonCamData)
        for j = 1:100
            R(:,k,j) = R2V(V2R(rotVec(camIdx{i},:)+randn(1,3).*rotVar(camIdx{i},:))/V2R(rotVec(nonCamIdx{k},:)+randn(1,3).*rotVar(nonCamIdx{k},:)));
        end
    end
    VR = var(R,[],3)';
    R = mean(R,3)';
    
    for j = 3:size(camData{i}.T_Skm1_Sk,1)
        
        VB12 = camData{i}.T_Var_Skm1_Sk(j-1,:) - [0,0,0,camData{i}.T_Var_Skm1_Sk(j-2,4),0,0,0];
        B12 = camData{i}.T_Skm1_Sk(j-1,:);
        
        VB23 = camData{i}.T_Var_Skm1_Sk(j,:) - [0,0,0,camData{i}.T_Var_Skm1_Sk(j-1,4),0,0,0];
        B23 = camData{i}.T_Skm1_Sk(j,:);
        
        eq1 = zeros(3,length(nonCamData));
        eq2 = zeros(3,length(nonCamData));
        eq3 = zeros(3,length(nonCamData));
        eq4 = zeros(3,length(nonCamData));
        
        for k = 1:length(nonCamData)
            VA12 = nonCamData{k}.T_Var_Skm1_Sk(j-1,:);
            A12 = nonCamData{k}.T_Skm1_Sk(j-1,:);

            VA23 = nonCamData{k}.T_Var_Skm1_Sk(j,:);
            A23 = nonCamData{k}.T_Skm1_Sk(j,:);

            eq1(:,k) = TEq(A23, B23, R(k,:));
            eq2(:,k) = TEq(A12, B12, R(k,:));
            eq3(:,k) = TEq2(B23);
            eq4(:,k) = TEq2(B12);
        end
        
        s(j-1) = (eq3(:)-eq4(:))\(eq1(:)-eq2(:));
        
        Vec = [A12,A23,B12,B23,R(1,:)];
        Var = [VA12,VA23,VB12,VB23,VR(1,:)];
        diff = 0.0001;
        vStore = zeros(size(Vec));
        
        %only bother with denominators
        for k = 1:31%[19,20,21,26,27,28]
            tV = Vec;
            tV(k) = tV(k) + sqrt(Var(k))*diff;

            A12 = tV(1:7);
            A23 = tV(8:14);
            B12 = tV(15:21);
            B23 = tV(22:28);
            Rs = tV(29:31);
            
            eq1 = TEq(A23, B23, Rs);
            eq2 = TEq(A12, B12, Rs);
            eq3 = TEq2(B23);
            eq4 = TEq2(B12);

            vStore(k) = (eq3-eq4)\(eq1-eq2) - s(j-1);
        end
        
        sV(j-1) = sum(real(vStore).^2,2)./(diff.^2);

    end
    
    s(1) = 0;
    s(end) = 0;
    sensorData{camIdx{i}}.T_Skm1_Sk(:,4) = s.*sensorData{camIdx{i}}.T_Skm1_Sk(:,4);
    sensorData{camIdx{i}}.T_Var_Skm1_Sk(:,4) = sV.*abs(sensorData{camIdx{i}}.T_Skm1_Sk(:,4));
end

warning('on','MATLAB:nearlySingularMatrix');

end