function [ prob ] = systemProb( sensorData, estVec )
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

prob = 0;
for a = 1:s
    for b = 1:s
        if(a < b)
            Rab = estMat{b}*estMat{a}';
            VA = sensorData{a}.T_Cov_Skm1_Sk(:,4:6)';
            VB = sensorData{b}.T_Cov_Skm1_Sk(:,4:6)';
            
            estA = sensorData{a}.T_Skm1_Sk(:,4:6)';
            estB = sensorData{b}.T_Skm1_Sk(:,4:6)';
            
            err = Rab*estA - estB;
            
            V = 1./sqrt([sum(repmat(Rab(1,:).^2,size(VA,2),1).*VA',2)+VB(1,1),...
            sum(repmat(Rab(2,:).^2,size(VA,2),1).*VA',2)+VB(2,2),...
            sum(repmat(Rab(3,:).^2,size(VA,2),1).*VA',2)+VB(3,3)]);
        
            temp = (err'.*V).^2;
            
            %temp = sort(temp);
            %temp = temp(1:floor(size(temp,1)*0.9));
            
            temp = sqrt(mean(temp(:))./mean(V(:).^2));
            
%              temp = zeros(size(VA,2),1);
%              for k = 1:size(VA,2)
%                 V = diag(VB(:,k)) + Rab*diag(VA(:,k))*Rab';
%                 temp(k) = err(:,k)'/V*err(:,k);
%              end

%            temp = cprobR(err, VA, VB, Rab);
%            temp = sum(sqrt(temp));
            
            prob = prob + temp;
        end
    end
end

end

