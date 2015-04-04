function [ varVec ] = ErrorEstCT2( sensorData, tranVec, rotVec, rotVar )
%OPTR Optimize translation based on inital guess
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   sensorData- nx1 cell containing sensor data sturcts
%   estVec- nx3 matrix of rotations for each sensor
%   rotVec- nx3 matrix of rotations for each sensor
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   outVec- nx3 matrix of the translation for each sensor
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
validateattributes(tranVec,{'numeric'},{'size',[length(sensorData),3]});
validateattributes(rotVec,{'numeric'},{'size',[length(sensorData),3]});

%pull usful info out of sensorData
TData = zeros(size(sensorData{1}.T_Skm1_Sk,1),6,length(sensorData));
vTData = TData;
s = zeros(length(sensorData),1);

for i = 1:length(sensorData)
    TData(:,:,i) = sensorData{i}.T_Skm1_Sk;
    vTData(:,:,i) = sensorData{i}.T_Var_Skm1_Sk;
    s(i) = strcmpi(sensorData{i}.type,'camera');
end

step = 0.000001;

tranVecS = tranVec(2:end,:);
rotVecS = rotVec;
TDataS = TData;
vTDataS = vTData;
sS = s;
varVec = zeros(size(tranVecS));

for x = 1:size(tranVecS,1)
    tranVec = tranVecS(x,:);
    TData = TDataS(:,:,1:2);
    TData(:,:,2) = TDataS(:,:,x+1);
    vTData = vTDataS(:,:,1:2);
    vTData(:,:,2) = vTDataS(:,:,x+1);
    rotVec = rotVecS(1:2,:);
    rotVec(2,:) = rotVecS(x+1,:);
    s = sS([1,x+1]);

    dxx = zeros(length(tranVec(:)));
    for i = 1:length(tranVec(:))
        for j = 1:length(tranVec(:))
            temp = tranVec; 
            temp(j) = temp(j) + step;
            temp(i) = temp(i) + step;
            f1 = SystemProbT(TData, vTData, s, temp, rotVec, rotVar, false);

            temp = tranVec; 
            temp(j) = temp(j) + step;
            temp(i) = temp(i) - step;
            f2 = SystemProbT(TData, vTData, s, temp, rotVec, rotVar, false);
            
            temp = tranVec; 
            temp(j) = temp(j) - step;
            temp(i) = temp(i) + step;
            f3 = SystemProbT(TData, vTData, s, temp, rotVec, rotVar, false);
            
            temp = tranVec; 
            temp(j) = temp(j) - step;
            temp(i) = temp(i) - step;
            f4 = SystemProbT(TData, vTData, s, temp, rotVec, rotVar, false);

            dxx(i,j) = (f1-f2-f3+f4)/(4*step*step);
        end
    end
    
    dxz = zeros(length(tranVec(:)),length(TData(:)));
    for i = 1:length(tranVec(:))
        temp = zeros(size(TData));
        for j = 1:size(TData,2)
            for k = 1:size(TData,3)
                tempA = tranVec; 
                tempA(i) = temp(i) + step;
                tempB = TData;
                tempB(:,j,k) = temp(:,j,k) + step;
                [f1,v1] = SystemProbT(tempB, vTData, s, tempA, rotVec, rotVar, true);

                tempA = tranVec; 
                tempA(i) = temp(i) - step;
                tempB = TData;
                tempB(:,j,k) = temp(:,j,k) + step;
                [f2,v2] = SystemProbT(tempB, vTData, s, tempA, rotVec, rotVar, true);
                
                tempA = tranVec; 
                tempA(i) = temp(i) + step;
                tempB = TData;
                tempB(:,j,k) = temp(:,j,k) - step;
                [f3,v3] = SystemProbT(tempB, vTData, s, tempA, rotVec, rotVar, true);
                
                tempA = tranVec; 
                tempA(i) = temp(i) - step;
                tempB = TData;
                tempB(:,j,k) = temp(:,j,k) - step;
                [f4,v4] = SystemProbT(tempB, vTData, s, tempA, rotVec, rotVar, true);

                %valid = and(and(v1,v2),and(v3,v4));
                temp(:,j,k) = (f1-f2-f3+f4)/(4*step*step);
                %temp(~valid,j,k) = 0;
            end
        end
        dxz(i,:) = temp(:);
    end

    d = dxx\dxz;
    d = (d.*repmat(vTData(:)',size(d,1),1))*d';
    varVec(x,:) = reshape(diag(d),3,[])';
end
    
varVec = [0,0,0;varVec];

end

