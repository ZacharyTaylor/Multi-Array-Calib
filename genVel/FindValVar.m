function [ tformVar ] = FindValVar( tform, velCurr, velPrev )
%FINDCAMVAR Summary of this function goes here
%   Detailed explanation goes here

%find variance
stepX = 0.001;
stepZ = 0.001;

velPT = (V2T(tform)*[velPrev, ones(size(velPrev,1),1)]')'; velPT = velPT(:,1:3);
idx = knnsearch(velCurr,velPT);
velCurr = velCurr(idx,:);

idx = knnsearch(velCurr,velCurr,'k',3);
nC = cross(velCurr(idx(:,2),:)-velCurr(idx(:,1),:),velCurr(idx(:,3),:)-velCurr(idx(:,1),:));
nC = nC./repmat(sqrt(sum(nC.^2,2)),1,3);
nC(isnan(nC)) = 0;

dxx = zeros(length(tform));
for i = 1:length(tform)
    for j = 1:length(tform)
        X = tform; 
        X(j) = X(j) + stepX;
        X(i) = X(i) + stepX;
        f1 = sum(findErr(X, velCurr, velPrev, nC));

        X = tform; 
        X(j) = X(j) + stepX;
        X(i) = X(i) - stepX;
        f2 = sum(findErr(X, velCurr, velPrev, nC));

        X = tform; 
        X(j) = X(j) - stepX;
        X(i) = X(i) + stepX;
        f3 = sum(findErr(X, velCurr, velPrev, nC));

        X = tform; 
        X(j) = X(j) - stepX;
        X(i) = X(i) - stepX;
        f4 = sum(findErr(X, velCurr, velPrev, nC));

        dxx(i,j) = (f1-f2-f3+f4)/(4*stepX*stepX);
    end
end

vel = velCurr;
vel(:,:,2) = velPrev;

dxz = zeros([length(tform),length(vel(:))]);
for i = 1:length(tform)
    dS = zeros(size(vel));
    for j = 1:size(vel,2)
        for k = 1:size(vel,3)

            X = tform; 
            X(i) = X(i) + stepX;
            Z = vel; 
            Z(:,j,k) = Z(:,j,k) + stepZ;
            f1 = findErr(X, Z(:,:,1),Z(:,:,2), nC);
            
            X = tform; 
            X(i) = X(i) - stepX;
            Z = vel; 
            Z(:,j,k) = Z(:,j,k) + stepZ;
            f2 = findErr(X, Z(:,:,1),Z(:,:,2), nC);
            
            X = tform; 
            X(i) = X(i) + stepX;
            Z = vel; 
            Z(:,j,k) = Z(:,j,k) - stepZ;
            f3 = findErr(X, Z(:,:,1),Z(:,:,2), nC);
            
            X = tform; 
            X(i) = X(i) - stepX;
            Z = vel; 
            Z(:,j,k) = Z(:,j,k) - stepZ;
            f4 = findErr(X, Z(:,:,1),Z(:,:,2), nC);
            
            dS(:,j,k) = (f1-f2-f3+f4)/(4*stepX*stepZ);
        end
    end
    dxz(i,:) = dS(:);
end

d = dxx\dxz;
d = 0.1*0.1*(d*d');

tformVar = diag(d)';

end

function [err] = findErr( tform, velCurr, velPrev, velCN)

tform = V2T(tform);
velPT = (tform*[velPrev, ones(size(velPrev,1),1)]')'; velPT = velPT(:,1:3);
err = sum((velCN.*(velPT - velCurr)).^2,2);
err(err > 0.2) = 0;

end

