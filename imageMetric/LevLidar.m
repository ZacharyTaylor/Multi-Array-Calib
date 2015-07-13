function [ points, valid ] = LevLidar(cloud)
%gets distance to each point

dist = sqrt(sum(cloud(:,1:3).^2,2));

diff = max(abs(dist(1:end-2)-dist(2:end-1)),abs(dist(2:end-1)-dist(3:end)));
diff = [0;diff;0];

valid = diff > 0.3;
points = [cloud(valid,1:3),sqrt(diff(valid))];

% y = 0.5;
% 
% sphere = zeros(size(cloud,1),2);
% sphere(:,1) = atan2(cloud(:,1), cloud(:,3));
% sphere(:,2) = atan(cloud(:,2)./ sqrt(cloud(:,1).^2 + cloud(:,3).^2));
% sphere(:,3) = sqrt(sum(cloud(:,1:3).^2,2));
% 
% idx = knnsearch(sphere(:,1:2),sphere(:,1:2),'k',9);
% 
% dis = abs(sphere(:,3) - mean(reshape(sphere(idx(:),3),size(idx)),2));
% 
% cloud = [cloud, dis];
% points = cloud(dis > 0.3,:);
% 
% %remove centre points to stop wrap round issue
% points = points(abs(points(:,2))> 0.2,:);



end

