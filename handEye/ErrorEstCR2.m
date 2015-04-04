function [ varVec ] = ErrorEstCR2( sensorData, rotVec )
%ERRORESTR estimate cramer rao lower bound for error variance
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   sensorData- nx1 cell containing sensor data sturcts
%   estVec- nx3 matrix of rotations for each sensor
%   step- step between test points for numercial differentiation
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   varVec- nx3 matrix containing rotational variance
%
%--------------------------------------------------------------------------
%   References:
%--------------------------------------------------------------------------
%   This function is part of the Multi-Array-Calib toolbox 
%   https://github.com/ZacharyTaylor/Multi-Array-Calib
%   
%   This code was written by Zachary Taylor
%   zacharyjeremytaylor@gmail.com
%   http://www.zjtaylor.com

%check inputs
validateattributes(sensorData,{'cell'},{'vector'});
for i = 1:length(sensorData)
    validateattributes(sensorData{i},{'struct'},{});
end
validateattributes(rotVec,{'numeric'},{'size',[length(sensorData),3]});

%pull usful info out of sensorData
RData = zeros(size(sensorData{1}.T_Skm1_Sk,1),3,length(sensorData));
vRData = RData;

for i = 1:length(sensorData)
    RData(:,:,i) = sensorData{i}.T_Skm1_Sk(:,4:6);
    vRData(:,:,i) = sensorData{i}.T_Var_Skm1_Sk(:,4:6);
end

step = 0.00001;

rotVecS = rotVec(2:end,:);
RDataS = RData;
vRDataS = vRData;
varVec = zeros(size(rotVecS));

for x = 1:size(rotVecS,1)
    rotVec = rotVecS(x,:);
    RData = RDataS(:,:,1:2);
    RData(:,:,2) = RDataS(:,:,x+1);
    vRData = vRDataS(:,:,1:2);
    vRData(:,:,2) = vRDataS(:,:,x+1);

    dxx = zeros(length(rotVec(:)));
    for i = 1:length(rotVec(:))
        for j = 1:length(rotVec(:))
            temp = rotVec; 
            temp(j) = temp(j) + step;
            temp(i) = temp(i) + step;
            f1 = SystemProbR(RData, vRData, temp, false);

            temp = rotVec; 
            temp(j) = temp(j) + step;
            temp(i) = temp(i) - step;
            f2 = SystemProbR(RData, vRData, temp, false);
            
            temp = rotVec; 
            temp(j) = temp(j) - step;
            temp(i) = temp(i) + step;
            f3 = SystemProbR(RData, vRData, temp, false);
            
            temp = rotVec; 
            temp(j) = temp(j) - step;
            temp(i) = temp(i) - step;
            f4 = SystemProbR(RData, vRData, temp, false);

            dxx(i,j) = (f1-f2-f3+f4)/(4*step*step);
        end
    end
    
    dxz = zeros(length(rotVec(:)),length(RData(:)));
    for i = 1:length(rotVec(:))
        temp = zeros(size(RData));
        for j = 1:size(RData,2)
            for k = 1:size(RData,3)
                tempA = rotVec; 
                tempA(i) = temp(i) + step;
                tempB = RData;
                tempB(:,j,k) = temp(:,j,k) + step;
                [f1,v1] = SystemProbR(tempB, vRData, tempA, true);

                tempA = rotVec; 
                tempA(i) = temp(i) - step;
                tempB = RData;
                tempB(:,j,k) = temp(:,j,k) + step;
                [f2,v2] = SystemProbR(tempB, vRData, tempA, true);
                
                tempA = rotVec; 
                tempA(i) = temp(i) + step;
                tempB = RData;
                tempB(:,j,k) = temp(:,j,k) - step;
                [f3,v3] = SystemProbR(tempB, vRData, tempA, true);
                
                tempA = rotVec; 
                tempA(i) = temp(i) - step;
                tempB = RData;
                tempB(:,j,k) = temp(:,j,k) - step;
                [f4,v4] = SystemProbR(tempB, vRData, tempA, true);

                %valid = and(and(v1,v2),and(v3,v4));
                temp(:,j,k) = (f1-f2-f3+f4)/(4*step*step);
                %temp(~valid,j,k) = 0;
            end
        end
        dxz(i,:) = temp(:);
    end

    d = dxx\dxz;
    d = (d.*repmat(vRData(:)',size(d,1),1))*d';
    varVec(x,:) = reshape(diag(d),3,[])';
end

varVec = [0,0,0;varVec];

end

