function [ points ] = removePointOverlap( points, angularRes )
%REMOVEPOINTOVERLAP Summary of this function goes here
%   Detailed explanation goes here

%velodyne angular res = 0.2 deg

angularRes = angularRes.*pi/180;

sphere = zeros(size(points,1),2);
sphere(:,1) = atan2(points(:,1), points(:,3));
sphere(:,2) = atan(points(:,2)./ sqrt(points(:,1).^2 + points(:,3).^2));

dist = sqrt(sum(points.^2,2));

idx = rangesearch(sphere,sphere,angularRes);

keep = true(size(points,1),1);

for i = 1:size(idx,1)
    r = dist(idx{i});
    for j = 1:size(r,1);
        if(r(j) > dist(i))
            keep(i) = false;
            break;
        end
    end
end

end

