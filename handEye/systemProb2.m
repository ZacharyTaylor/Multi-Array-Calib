function [ prob ] = systemProb2( sensorData, estVec )
%ROTSYS2VEC Summary of this function goes here
%   Detailed explanation goes here

estVec = [0,0,0;estVec];

s = size(sensorData,1);
estMat = cell(s,1);

%fill base values
estMat{1} = eye(3);
for i = 2:s
    estMat{i} = vec2rot(estVec(i,:)');
end

err = zeros(size(sensorData{1}.T_Skm1_Sk,1),3,length(sensorData));
Vo = zeros(3,3,size(sensorData{1}.T_Skm1_Sk,1),length(sensorData));
for a = 1:s
    R = inv(estMat{a});
    V = sensorData{a}.T_Cov_Skm1_Sk(:,4:6);
    est = sensorData{a}.T_Skm1_Sk(:,4:6);
    err(:,:,a) = (R*est')';

    Vo(:,:,:,a) = rotV(V,R);
end

m = dV(sum(Vo,4),sum(eV(Vo,err),3));

err = eV(Vo,(err - repmat(m,1,1,size(err,3))));
prob = sqrt(mean(err(:).^2));%/mean(Vo(:).^2));

end

