function [ tformVar ] = FindCamVar( tform, pNew, pOld, K )
%FINDCAMVAR Summary of this function goes here
%   Detailed explanation goes here

%find variance
step = 0.001;

dxx = zeros(length(tform));
for i = 1:length(tform)
    for j = 1:length(tform)
        temp = tform; 
        temp(j) = temp(j) + step;
        temp(i) = temp(i) + step;
        f1 = sum(findErr(temp, pNew, pOld, K));

        temp = tform; 
        temp(j) = temp(j) + step;
        temp(i) = temp(i) - step;
        f2 = sum(findErr(temp, pNew, pOld, K));

        temp = tform; 
        temp(j) = temp(j) - step;
        temp(i) = temp(i) + step;
        f3 = sum(findErr(temp, pNew, pOld, K));

        temp = tform; 
        temp(j) = temp(j) - step;
        temp(i) = temp(i) - step;
        f4 = sum(findErr(temp, pNew, pOld, K));

        dxx(i,j) = (f1-f2-f3+f4)/(4*step*step);
    end
end

dx = zeros(length(tform),1);
for i = 1:length(tform)
    temp = tform; 
    temp(i) = temp(i) + step;
    f1 = sum(findErr(temp, pNew, pOld, K));
        
    temp = tform; 
    temp(i) = temp(i) - step;
    f2 = sum(findErr(temp, pNew, pOld, K));

    dx(i) = (f1-f2)/(2*step);
end

p = pNew;
p(:,:,2) = pOld;

dz = zeros(size(p));
for i = 1:size(p,2)
    for j = 1:size(p,3)
        temp = p; 
        temp(:,i,j) = temp(:,i,j) + step;
        f1 = findErr(tform, temp(:,:,1), temp(:,:,2), K);

        temp = p; 
        temp(:,i,j) = temp(:,i,j) - step;
        f2 = findErr(tform, temp(:,:,1),temp(:,:,2), K);

        dz(:,i) = (f1-f2)/(2*step);
    end
end
dz = dz(:);

dxz = zeros(size(dx(:),1),size(dz(:),1));
for i = 1:size(dx(:),1)
    for j = 1:size(dz(:),1)
        dxz(i,j) = dx(i) + dz(j);
    end
end

d = dxx\dxz;
d = 0.5*0.5*(d*d');

tformVar = diag(d)';


end

function [err] = findErr( tform, pNew, pOld, K )

tform = V2T(tform);
R = tform(1:3,1:3);
tx = [0, -tform(3,4), tform(2,4); tform(3,4), 0, -tform(1,4); -tform(2,4), tform(1,4), 0];

%find essintial matrix
E = R*tx;

%find fundemental matrix
K = K(1:3,1:3);
F = K'\(E/K);
F = F/norm(F);

%find error
err = zeros(size(pNew,1),1);
for i = 1:size(err,1)
    err(i) = [pNew(i,:),1]*F*[pOld(i,:),1]';
end

err = err.^2;

end

