function [ varVec ] = ErrorEstT( sensorData, tranVec, rotVec )
%ERRORESTT uses variance to find a measure of the systems probablity of
%   being correct (lower score == better)
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   sensorData- nx1 cell containing sensor data sturcts
%   tranVec- nx3 matrix of rotations for each sensor
%   rotVec- nx3 matrix of rotations for each sensor
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   varVec- nx3 matrix containing translational variance
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

%convert rot vector to rotmats
rotMat = zeros(3,3,size(sensorData,1));
for i = 1:size(rotMat,3)
    rotMat(:,:,i) = V2R(rotVec(i,:));
end

%get matrix form of transformations
tformMat = cell(size(sensorData));
for i = 1:size(sensorData,1)
    tformMat{i} = zeros(size(sensorData{i}.T_Skm1_Sk,1),12);
    for j = 1:size(sensorData{i}.T_Skm1_Sk,1)
        temp = V2T(sensorData{i}.T_Skm1_Sk(j,:));
        r = temp(1:3,1:3);
        t = temp(1:3,4);
        tformMat{i}(j,:) = [r(:)' t(:)'];
    end
end

varVec = zeros(length(sensorData),3);

a = 1;
for b = 1:length(sensorData)
    if(a < b)
        Ta = [rotMat(:,:,a),tranVec(a,:)';[0,0,0,1]];
        Tb = [rotMat(:,:,b),tranVec(b,:)';[0,0,0,1]];
        Tab = Tb/Ta;

        Rab = Tab(1:3,1:3);
        temp = Tab(1:3,4);

        VA = sensorData{a}.T_Var_Skm1_Sk(:,1:3)';
        VB = sensorData{b}.T_Var_Skm1_Sk(:,1:3)';

        estA = sensorData{a}.T_Skm1_Sk(:,1:3)';
        estB = sensorData{b}.T_Skm1_Sk(:,1:3)';

        estA = -Rab*estA;

        estAB = tformMat{b}(:,1:9);
        estAB(:,1) = estAB(:,1) - 1;
        estAB(:,5) = estAB(:,5) - 1;
        estAB(:,9) = estAB(:,9) - 1;

        estAB = [estAB(:,1)*temp(1) + estAB(:,4)*temp(2) + estAB(:,7)*temp(3), estAB(:,2)*temp(1) + estAB(:,5)*temp(2) + estAB(:,8)*temp(3), estAB(:,3)*temp(1) + estAB(:,6)*temp(2) + estAB(:,9)*temp(3)];

        if(strcmp(sensorData{a}.type,'camera'))
            if(strcmp(sensorData{b}.type,'camera'))
                temp = (estA(1,:).^2.*estB(2,:).^2 + estA(1,:).^2.*estB(3,:).^2 - 2.*estA(1,:).*estA(2,:).*estB(1,:).*estB(2,:) - 2.*estA(1,:).*estA(3,:).*estB(1,:).*estB(3,:) + estA(2,:).^2.*estB(1,:).^2 + estA(2,:).^2.*estB(3,:).^2 - 2.*estA(2,:).*estA(3,:).*estB(2,:).*estB(3,:) + estA(3,:).^2.*estB(1,:).^2 + estA(3,:).^2.*estB(2,:).^2);

                tempA = sum(estA.*[err(1,:).*estB(2,:).^2 - estB(1,:).*err(2,:).*estB(2,:) + err(1,:).*estB(3,:).^2 - estB(1,:).*err(3,:).*estB(3,:); err(2,:).*estB(1,:).^2 - estB(2,:).*err(1,:).*estB(1,:) + err(2,:).*estB(3,:).^2 - estB(2,:).*err(3,:).*estB(3,:); err(3,:).*estB(1,:).^2 - estB(3,:).*err(1,:).*estB(1,:) + err(3,:).*estB(2,:).^2 - estB(3,:).*err(2,:).*estB(2,:)])./temp;
                tempA = repmat(tempA,3,1);
                tempB = sum(estB.*[err(1,:).*estA(2,:).^2 - estA(1,:).*err(2,:).*estA(2,:) + err(1,:).*estA(3,:).^2 - estA(1,:).*err(3,:).*estA(3,:); err(2,:).*estA(1,:).^2 - estA(2,:).*err(1,:).*estA(1,:) + err(2,:).*estA(3,:).^2 - estA(2,:).*err(3,:).*estA(3,:); err(3,:).*estA(1,:).^2 - estA(3,:).*err(1,:).*estA(1,:) + err(3,:).*estA(2,:).^2 - estA(3,:).*err(2,:).*estA(2,:)])./temp;
                tempB = repmat(tempB,3,1);

                VA = VA.*abs(tempA);
                VB = VB.*abs(tempB);

                err = err - tempA.*estA - tempB.*estB;

            else
                err = -estAB' - estB;

                temp = sum(err.*estA)./sum(estA.^2);
                temp = repmat(temp,3,1);

                VA = abs(temp).*VA;
                err = err - temp.*estA;  
            end

        elseif(strcmp(sensorData{b}.type,'camera'))
            err = -estAB' - estA;

            temp = sum(err.*estB)./sum(estB.^2);
            temp = repmat(temp,3,1);

            VB = abs(temp).*VB;
            err = err - temp.*estB;    
        else               
            err = estAB' + estA + estB;
        end

        err = diag(var(err'))/(estAB'*estAB);
        varVec(b,:) = diag(err);

    end
end

end

