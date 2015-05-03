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

stepX = 0.0000001;
stepZ = 0.0000001;

rotVec = rotVec(2:end,:);
varVec = zeros(size(rotVec));

rotVecS = rotVec;
RDataS = RData;
vRDataS = vRData;

for x = 2:size(RDataS,3)
    
    rotVec = rotVecS(x-1,:);
    RData = RDataS(:,:,[1,x]);
    vRData = vRDataS(:,:,[1,x]);

    dxx = zeros(length(rotVec(:)));
    for i = 1:length(rotVec(:))
        for j = 1:length(rotVec(:))
            X = rotVec; 
            X(i) = X(i) + stepX;
            X(j) = X(j) + stepX;
            f1 = SystemProbR(RData, vRData, X, false);

            X = rotVec; 
            X(i) = X(i) + stepX;
            X(j) = X(j) - stepX;
            f2 = SystemProbR(RData, vRData, X, false);

            X = rotVec; 
            X(i) = X(i) - stepX;
            X(j) = X(j) + stepX;
            f3 = SystemProbR(RData, vRData, X, false);

            X = rotVec; 
            X(i) = X(i) - stepX;
            X(j) = X(j) - stepX;
            f4 = SystemProbR(RData, vRData, X, false);

            dxx(i,j) = (f1-f2-f3+f4)/(4*stepX*stepX);
        end
    end

    dxz = zeros(length(rotVec(:)),length(RData(:)));
    for i = 1:length(rotVec(:))
        dS = zeros(size(RData));
        for j = 1:size(RData,2)
            for k = 1:size(RData,3)
                X = rotVec; 
                X(i) = X(i) + stepX;
                Z = RData;
                Z(:,j,k) = Z(:,j,k) + stepZ;
                f1 = SystemProbR(Z, vRData, X, true);

                X = rotVec; 
                X(i) = X(i) + stepX;
                Z = RData;
                Z(:,j,k) = Z(:,j,k) - stepZ;
                f2 = SystemProbR(Z, vRData, X, true);

                X = rotVec; 
                X(i) = X(i) - stepX;
                Z = RData;
                Z(:,j,k) = Z(:,j,k) + stepZ;
                f3 = SystemProbR(Z, vRData, X, true);

                X = rotVec; 
                X(i) = X(i) - stepX;
                Z = RData;
                Z(:,j,k) = Z(:,j,k) - stepZ;
                f4 = SystemProbR(Z, vRData, X, true);

                v = and(and(f1,f2),and(f3,f4));
                dS(v,j,k) = (f1(v)-f2(v)-f3(v)+f4(v))/(4*stepX*stepZ);
            end
        end
        dxz(i,:) = dS(:);
    end

    d = dxx\dxz;
    d = (d.*repmat(vRData(:)',size(d,1),1))*d';
    d = reshape(diag(d),3,[])';

    varVec(x-1,:) = d;
end

varVec = [0,0,0;varVec];

end

