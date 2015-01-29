function [ errVec ] = errorEst( sensorData, estVec )
%ROTSYS2VEC Summary of this function goes here
%   Detailed explanation goes here

s = size(sensorData,1);
estMat = cell(s,1);

%fill base values
estMat{1} = eye(3);
for i = 2:s
    estMat{i} = vec2rot(estVec(i,:)');
end

errVec = zeros(s,3);

a = 1;
for b = 1:s
    if(a < b)
        Rab = estMat{b}*estMat{a}';
        VA = sensorData{a}.T_Cov_Skm1_Sk(:,4:6)';
        VB = sensorData{b}.T_Cov_Skm1_Sk(:,4:6)';

        estA = sensorData{a}.T_Skm1_Sk(:,4:6)';
        estB = sensorData{b}.T_Skm1_Sk(:,4:6)';

        err = Rab*estA - estB;
        err = diag(var(err'))/(estA*estA');
        errVec(b,:) = diag(err);
    end
end

end

