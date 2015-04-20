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

stepX = 0.00001;
stepZ = 0.00001;

tranVec = tranVec(2:end,:);

dxx = zeros(length(tranVec(:)));
for i = 1:length(tranVec(:))
    for j = 1:length(tranVec(:))
        X = tranVec;
        X(i) = X(i) + stepX;
        X(j) = X(j) + stepX;
        f1 = SystemProbT(TData, vTData, s, X, rotVec, rotVar, false);

        X = tranVec;
        X(i) = X(i) - stepX;
        X(j) = X(j) + stepX;
        f2 = SystemProbT(TData, vTData, s, X, rotVec, rotVar, false);
        
        X = tranVec;
        X(i) = X(i) + stepX;
        X(j) = X(j) - stepX;
        f3 = SystemProbT(TData, vTData, s, X, rotVec, rotVar, false);
        
        X = tranVec;
        X(i) = X(i) - stepX;
        X(j) = X(j) - stepX;
        f4 = SystemProbT(TData, vTData, s, X, rotVec, rotVar, false);

        dxx(i,j) = (f1-f2-f3+f4)/(4*stepX*stepX);
    end
end

dxz = zeros(length(tranVec(:)),length(TData(:)));
for i = 1:length(tranVec(:))
    dS = zeros(size(TData));
    for j = 1:size(TData,2)
        for k = 1:size(TData,3)
            X = tranVec; 
            X(i) = X(i) + stepX;
            Z = TData;
            Z(:,j,k) = Z(:,j,k) + stepZ;
            f1 = SystemProbT(Z, vTData, s, X, rotVec, rotVar, true);

            X = tranVec; 
            X(i) = X(i) + stepX;
            Z = TData;
            Z(:,j,k) = Z(:,j,k) - stepZ;
            f2 = SystemProbT(Z, vTData, s, X, rotVec, rotVar, true);
            
            X = tranVec; 
            X(i) = X(i) - stepX;
            Z = TData;
            Z(:,j,k) = Z(:,j,k) + stepZ;
            f3 = SystemProbT(Z, vTData, s, X, rotVec, rotVar, true);
            
            X = tranVec; 
            X(i) = X(i) - stepX;
            Z = TData;
            Z(:,j,k) = Z(:,j,k) - stepZ;
            f4 = SystemProbT(Z, vTData, s, X, rotVec, rotVar, true);

            dS(:,j,k) = (f1-f2-f3+f4)/(4*stepX*stepZ);
        end
    end
    dxz(i,:) = dS(:);
end

d = dxx\dxz;
d = (d.*repmat(vTData(:)',size(d,1),1))*d';
varVec = reshape(diag(d),3,[])';

varVec = [0,0,0;varVec];

end

