function [ sensorData ] = SolveScale( sensorData, rotVec, rotVar )

%divide into camera and non-camera sensors
nonCamData = {};
nonCamIdx = {};
camData = {};
camIdx = {};

warning('off','MATLAB:nearlySingularMatrix');
samples = 100;

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
    
    s = ones(size(camData{i}.T_Skm1_Sk,1),samples);
    
    for j = 3:size(camData{i}.T_Skm1_Sk,1)
        for k = 1:samples
            vals = sqrt(nonCamData{1}.T_Var_Skm1_Sk(j-1));
            vals = nonCamData{1}.T_Skm1_Sk(j-1,:)+randn(1,7).*vals;
            Ta12 = V2T(vals);
            ra12 = Ta12(1:3,1:3);
            ta12 = Ta12(1:3,4);

            vals = sqrt(camData{i}.T_Var_Skm1_Sk(j-1) - [0,0,0,camData{i}.T_Var_Skm1_Sk(j-2),0,0,0]);
            vals = camData{i}.T_Skm1_Sk(j-1,:)+randn(1,7).*vals;
            Tb12 = V2T(vals);
            rb12 = Tb12(1:3,1:3);
            tb12 = Tb12(1:3,4);

            vals = sqrt(nonCamData{1}.T_Var_Skm1_Sk(j));
            vals = nonCamData{1}.T_Skm1_Sk(j,:)+randn(1,7).*vals;
            Ta23 = V2T(vals);
            ra23 = Ta23(1:3,1:3);
            ta23 = Ta23(1:3,4);

            vals = sqrt(camData{i}.T_Var_Skm1_Sk(j) - [0,0,0,camData{i}.T_Var_Skm1_Sk(j-1),0,0,0]);
            vals = camData{i}.T_Skm1_Sk(j,:)+randn(1,7).*vals;
            Tb23 = V2T(vals);
            rb23 = Tb23(1:3,1:3);
            tb23 = Tb23(1:3,4);

            R = V2R(rotVec(camIdx{i},:))/V2R(rotVec(nonCamIdx{i},:));

            eq1 = (rb23-eye(3))\(R*ta23);
            eq2 = (rb12-eye(3))\(R*ta12);
            eq3 = (rb23-eye(3))\tb23;
            eq4 = (rb12-eye(3))\tb12;

            s(j-1,k) = (eq3-eq4)\(eq1-eq2);
        end
    end
    
    sensorData{camIdx{i}}.T_Skm1_Sk(:,4) = s.*sensorData{camIdx{i}}.T_Skm1_Sk(:,4);
end

warning('on','MATLAB:nearlySingularMatrix');

end