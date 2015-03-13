function [ sensorData ] = SolveScale3( sensorData, rotVec, rotVar, wind )

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
    sV = ones(size(camData{i}.T_Skm1_Sk,1),11);
    
    samples = 100;
    R = zeros(3,length(nonCamData),samples);
    for k = 1:length(nonCamData)
        for j = 1:100
            R(:,k,j) = R2V(V2R(rotVec(camIdx{i},:)+randn(1,3).*rotVar(camIdx{i},:))/V2R(rotVec(nonCamIdx{k},:)+randn(1,3).*rotVar(nonCamIdx{k},:)));
        end
    end
    VR = var(R,[],3)';
    R = mean(R,3)';
    
    for j = 1:size(camData{i}.T_Skm1_Sk,1)
        
        %get all elements inside window
        lower = max(1,j-wind);
        upper = min(size(camData{i}.T_Skm1_Sk,1),j+wind);
        
        A = nonCamData{1}.T_Skm1_Sk(lower:upper,:);
        vA = nonCamData{1}.T_Var_Skm1_Sk(lower:upper,:);
        %temp = abs(vA(wind,4) - vA(wind-1,4));
        %vA(:,4) = abs(vA(:,4)-repmat(vA(wind,4),upper-lower+1,1));
        %vA(wind,4) = temp;
        
        B = camData{i}.T_Skm1_Sk(lower:upper,:);
        vB = camData{i}.T_Var_Skm1_Sk(lower:upper,:);
        %temp = abs(vB(wind,4) - vB(wind-1,4));
        %vB(:,4) = abs(vB(:,4)-repmat(vB(wind,4),upper-lower+1,1));
        %vB(wind,4) = temp;
        
        eq1 = zeros(3*(upper-lower+1),4);
        eq2 = zeros(3*(upper-lower+1),1);
        for k = 1:(upper-lower+1)
            eq1((3*(k-1)+1):3*k,:) = (1/vB(k,4)).*[V2R(B(k,5:7))-eye(3),B(k,1:3)'*B(k,4)];
            eq2((3*(k-1)+1):3*k,:) = (1/vB(k,4)).*V2R(R(1,:))*(A(k,1:3)*A(k,4))';
            %eq1((3*(k-1)+1):3*k,:) = [V2R(B(k,5:7))-eye(3),B(k,1:3)'*B(k,4)];
            %eq2((3*(k-1)+1):3*k,:) = V2R(R(1,:))*(A(k,1:3)*A(k,4))';
        end
            
        temp = eq1\eq2;
        s(j) = temp(4);
        
%         %find variance
%         diff = 0.01;
%         for m = 1:7
%             sB = B;
%             sB(:,m) = sB(:,m) + sqrt(vB(:,m)).*diff.*randi(size(sB,1),1);
%             for k = 1:(upper-lower+1)
%                 eq1((3*(k-1)+1):3*k,:) = (1/vB(k,4)).*[V2R(sB(k,5:7))-eye(3),sB(k,1:3)'*sB(k,4)];
%                 eq2((3*(k-1)+1):3*k,:) = (1/vB(k,4)).*V2R(R(1,:))*(A(k,1:3)*A(k,4))';
%             end
%             temp = eq1\eq2;
%             sV(j,m) = temp(4);
%         end
%         
%         for m = 1:4
%             sA = A;
%             sA(:,m) = sA(:,m) + sqrt(vA(:,m)).*diff.*randi(size(sA,1),1);
%             for k = 1:(upper-lower+1)
%                 eq1((3*(k-1)+1):3*k,:) = (1/vB(k,4)).*[V2R(B(k,5:7))-eye(3),B(k,1:3)'*B(k,4)];
%                 eq2((3*(k-1)+1):3*k,:) = (1/vB(k,4)).*V2R(R(1,:))*(sA(k,1:3)*sA(k,4))';
%             end
%             temp = eq1\eq2;
%             sV(j,m+7) = temp(4);
%         end
         
    end
    
%     sV = ((sV - repmat(s,1,11)).^2)./(diff.^2);
%     sV = sum(sV,2);
    
    s(1) = 0;
    s(end) = 0;
    sensorData{camIdx{i}}.T_Var_Skm1_Sk(:,4) = s.*abs(sensorData{camIdx{i}}.T_Var_Skm1_Sk(:,4));
    sensorData{camIdx{i}}.T_Skm1_Sk(:,4) = s.*sensorData{camIdx{i}}.T_Skm1_Sk(:,4);
end

warning('on','MATLAB:nearlySingularMatrix');

end