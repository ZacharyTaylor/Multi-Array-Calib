function [ tformVar ] = FindCamVar( tform, pNew, pOld, K )
%FINDCAMVAR Summary of this function goes here
%   Detailed explanation goes here

%find variance
stepX = 0.001;
stepZ = 1;

dxx = zeros(length(tform));
for i = 1:length(tform)
    for j = 1:length(tform)
        X = tform; 
        X(j) = X(j) + stepX;
        X(i) = X(i) + stepX;
        f1 = sum(findErr(X, pNew, pOld, K));

        X = tform; 
        X(j) = X(j) + stepX;
        X(i) = X(i) - stepX;
        f2 = sum(findErr(X, pNew, pOld, K));

        X = tform; 
        X(j) = X(j) - stepX;
        X(i) = X(i) + stepX;
        f3 = sum(findErr(X, pNew, pOld, K));

        X = tform; 
        X(j) = X(j) - stepX;
        X(i) = X(i) - stepX;
        f4 = sum(findErr(X, pNew, pOld, K));

        dxx(i,j) = (f1-f2-f3+f4)/(4*stepX*stepX);
    end
end

p = pNew;
p(:,:,2) = pOld;

dxz = zeros([length(tform),length(p(:))]);
for i = 1:length(tform)
    dS = zeros(size(p));
    for j = 1:size(p,2)
        for k = 1:size(p,3)

            X = tform; 
            X(i) = X(i) + stepX;
            Z = p; 
            Z(:,j,k) = Z(:,j,k) + stepZ;
            f1 = findErr(X, Z(:,:,1),Z(:,:,2), K);
            
            X = tform; 
            X(i) = X(i) - stepX;
            Z = p; 
            Z(:,j,k) = Z(:,j,k) + stepZ;
            f2 = findErr(X, Z(:,:,1),Z(:,:,2), K);
            
            X = tform; 
            X(i) = X(i) + stepX;
            Z = p; 
            Z(:,j,k) = Z(:,j,k) - stepZ;
            f3 = findErr(X, Z(:,:,1),Z(:,:,2), K);
            
            X = tform; 
            X(i) = X(i) - stepX;
            Z = p; 
            Z(:,j,k) = Z(:,j,k) - stepZ;
            f4 = findErr(X, Z(:,:,1),Z(:,:,2), K);
            
            dS(:,j,k) = (f1-f2-f3+f4)/(4*stepX*stepZ);
        end
    end
    dxz(i,:) = dS(:);
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
err = F*[pOld,ones(size(pOld,1),1)]';
err = sum([pNew,ones(size(pOld,1),1)].*err',2);
err = err.^2;

end

