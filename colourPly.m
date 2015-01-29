function [ out ] = colourPly( sensorData, lidar, tform, frame )
%COLOURPLY Summary of this function goes here
%   Detailed explanation goes here

%hidden = findHidden(lidar, vec2tran(tform'));

image = imread([sensorData.folder  sensorData.files(frame).name]);
image = double(image)/255;
for i = 1:size(image,3)
    image(:,:,i) = undistort(image(:,:,i), sensorData.D, sensorData.K);
end
imageSize = size(image);
tform = gpuArray(single(vec2tran(tform')));
cam = gpuArray(single(sensorData.K));
[proj,valid] = projectLidar( tform, cam, lidar, imageSize(1:2));
proj = proj(valid,:);

temp = interpolateImage(gpuArray(single(image)), proj);
if(size(temp,2) == 1)
    temp = [temp,temp,temp];
end

out = zeros(size(lidar,1),3);
out(gather(valid),:) = gather(temp);
%out(hidden,1) = 0;
%out(hidden,2) = 0;
%out(hidden,3) = 0;

end

