function [ varOff ] = FindTimingVar( offsets, t, Mag, Var, navFlag, dTest, samples )
%FINDTIMINGVAR Summary of this function goes here
%   Detailed explanation goes here

stepX = repmat([1000,0.0001],size(offsets,1));
stepZ = 0.001;

%get variance
dxx = zeros(length(offsets(:)));
for i = 1:length(offsets(:))
    for j = 1:length(offsets(:))
        
        X = offsets;
        X(i) = X(i) + stepX(i);
        X(j) = X(j) + stepX(j);
        f1 = sum(GetTimingError(t,Mag,Var,samples,X, navFlag, dTest));

        X = offsets;
        X(i) = X(i) + stepX(i);
        X(j) = X(j) - stepX(j);
        f2 = sum(GetTimingError(t,Mag,Var,samples,X, navFlag, dTest));

        X = offsets;
        X(i) = X(i) - stepX(i);
        X(j) = X(j) + stepX(j);
        f3 = sum(GetTimingError(t,Mag,Var,samples,X, navFlag, dTest));

        X = offsets;
        X(i) = X(i) - stepX(i);
        X(j) = X(j) - stepX(j);
        f4 = sum(GetTimingError(t,Mag,Var,samples,X, navFlag, dTest));

        dxx(i,j) = (f1-f2-f3+f4)/(4*stepX(i)*stepX(j));
    end
end

dxz = zeros([length(offsets(:)),samples*length(Mag)]);
for i = 1:length(offsets(:))
    dS = cell(size(Mag(:)));
    for j = 1:size(Mag(:),1)
               
        X = offsets; 
        X(i) = X(i) + stepX(i);
        Z = dTest; 
        Z{j} = Z{j} + stepZ;
        f1 = GetTimingError(t,Mag,Var,samples,X, navFlag, Z);
        
        X = offsets; 
        X(i) = X(i) + stepX(i);
        Z = dTest; 
        Z{j} = Z{j} - stepZ;
        f2 = GetTimingError(t,Mag,Var,samples,X, navFlag, Z);

        X = offsets; 
        X(i) = X(i) - stepX(i);
        Z = dTest; 
        Z{j} = Z{j} + stepZ;
        f3 = GetTimingError(t,Mag,Var,samples,X, navFlag, Z);
        
        X = offsets; 
        X(i) = X(i) - stepX(i);
        Z = dTest; 
        Z{j} = Z{j} - stepZ;
        f4 = GetTimingError(t,Mag,Var,samples,X, navFlag, Z);

        dS{j} = (f1-f2-f3+f4)/(4*stepX(i)*stepZ);
    end
    dxz(i,:) = cell2mat(dS');
end

%find overlapping regions
tMin = 0;
tMax = inf;
for k = 1:length(t)
    tMin = max(tMin,t{k}(1));
    tMax = min(tMax,t{k}(end));
end

%sample overlapping area
tI = tMin:(tMax-tMin)/samples:tMax; tI = tI(2:end);

for k = 1:length(t)
    Var{k} = interp1(t{k},Var{k},tI,'pchip');
    if(navFlag(k))
        Var{k} = abs(Var{k});
    else
        Var{k} = [0,abs(diff(Var{k}))];
    end
end

v = cell2mat(Var');

d = dxx\dxz;
d = (d.*repmat(v(:)',size(d,1),1))*d';
varOff = reshape(diag(d),[],2);

end

