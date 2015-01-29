function [ points ] = thinLidar(cloud)
%gets distance to each point

% y = 0.5;
% 
% cloud(:,5) = sqrt(cloud(:,1).^2+cloud(:,2).^2 + cloud(:,3).^2);
% 
% points = cloud(2:end-1,:);
% points(:,4) = max(abs(cloud(1:end-2,4) - cloud(2:end-1,4)), abs(cloud(3:end,4) - cloud(2:end-1,4)));
% % points(:,5) = points(:,5).^y;
% % 
% % points = points(points(:,5) > 0.3,:);
% % 
% 
% %points = cloud;
% [~,idx] = sort(points(:,4));
% idx = idx(floor(size(idx,1)*0.9):end);
% points = points(idx,1:4);

points = datasample(cloud,10000,1,'Replace',false);

end

