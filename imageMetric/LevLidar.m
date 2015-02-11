function [ points ] = LevLidar(cloud)
%gets distance to each point

y = 0.5;

sphere = zeros(size(cloud,1),2);
sphere(:,1) = atan2(cloud(:,1), cloud(:,3));
sphere(:,2) = atan(cloud(:,2)./ sqrt(cloud(:,1).^2 + cloud(:,3).^2));
sphere(:,3) = sqrt(sum(cloud(:,1:3).^2,2));

idx = knnsearch(sphere(:,1:2),sphere(:,1:2),'k',9);

dis = abs(sphere(:,3) - mean(reshape(sphere(idx(:),3),size(idx)),2))./sphere(:,3);

cloud = [cloud, dis];
points = cloud(dis > 0.03,:);

%remove centre points to stop wrap round issue
points = points(abs(points(:,2))> 0.2,:);

end

