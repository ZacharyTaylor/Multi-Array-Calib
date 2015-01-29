function [ data ] = GetNorms2(data)
%remove non distance related points
cloud = data(:,1:3);
data(end,7) = 0;
cloud(:,4) = 0;

%transform points

cloud = cloud(:,1:4);
cloud(:,4) = 1;

%project points onto sphere
sphere = cloud; zeros(size(cloud,1),3);
sphere(:,1) = atan2(cloud(:,1), cloud(:,3));
sphere(:,2) = atan(cloud(:,2)./ sqrt(cloud(:,1).^2 + cloud(:,3).^2));
sphere(:,3) = sqrt(cloud(:,1).^2 + cloud(:,2).^2 + cloud(:,3).^2);
sphere(:,3) = sphere(:,3) ./ max(sphere(:,3));

numNeighbours = 9;

%create kdtree
kdtreeobj = KDTreeSearcher(sphere(:,1:2),'distance','euclidean');

%get nearest neighbours
n = knnsearch(kdtreeobj,sphere(:,1:2),'k',(numNeighbours+1));

%remove self
n = n(:,2:end);

p = repmat(sphere(:,1:3),numNeighbours,1) - sphere(n(:),1:3);
p = reshape(p, size(sphere,1),numNeighbours,3);

C = zeros(size(sphere,1),6);
C(:,1) = sum(p(:,:,1).*p(:,:,1),2);
C(:,2) = sum(p(:,:,1).*p(:,:,2),2);
C(:,3) = sum(p(:,:,1).*p(:,:,3),2);
C(:,4) = sum(p(:,:,2).*p(:,:,2),2);
C(:,5) = sum(p(:,:,2).*p(:,:,3),2);
C(:,6) = sum(p(:,:,3).*p(:,:,3),2);
C = C ./ numNeighbours;
C(~isfinite(C)) = 0;

for i = 1:(size(sphere,1)-1)
    Cmat = [C(i,1) C(i,2) C(i,3); C(i,2) C(i,4) C(i,5); C(i,3) C(i,5) C(i,6)];
    
    %get eigen values and vectors
    [v,d] = eig(Cmat);
    d = diag(d);
    
    [~,k] = min(d);
    norm = v(:,k);
    
    %get norm quality
    q = d / d(k);
    q = q(q ~= 1);
    q = max(q);
    
    %store normal values
    %data(i,4) = abs(atan2(abs(norm(1)),sqrt(norm(2)^2 + norm(3)^2)));
    
    data(i,4:6) = abs(norm);
    data(i,7) = q;
end

data(~isfinite(data(:))) = 0;
data = data(any(abs(data(:,4:6)) < 1,2),:);

data = data(data(:,7) > 5,:);
[~,data(:,4:6)] = pca(data(:,4:6));
data = data(:,1:4); 
 
data(:,4) = data(:,4) - min(data(:,4));
data(:,4) = data(:,4) / max(data(:,4));

data(~isfinite(data(:))) = 0;

end