function [ hidden ] = findHidden( lidar, tform )
%REMOVEHIDDEN Summary of this function goes here
%   Detailed explanation goes here

lidar = gather(lidar);

%velodyne point resolution
res = 0.2*pi/180;

%transform points
lidar = (tform*[lidar(:,1:3), ones(size(lidar,1),1)]')';

%project onto sphere
sphere = zeros(size(lidar,1),2);
sphere(:,1) = atan2(lidar(:,1),lidar(:,3));
sphere(:,2) = atan2(lidar(:,2),sqrt(lidar(:,1).^2 + lidar(:,2).^2));

dist = sqrt(sum(lidar(:,1:3).^2,2));

hidden = false(size(lidar,1),1);

idx = rangesearch(sphere,sphere,res);
for i = 1:size(idx,1)
    for j = 1:size(idx{i},2)
        if((dist(i) - 1) > dist(idx{i}(j)))
            hidden(i) = true;
            break;
        end
    end
end

end

