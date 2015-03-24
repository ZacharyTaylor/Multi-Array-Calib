function [ error ] = runFlowMetricBoot( tform, K, scans, images, boot, invert )
%RUNFLOWMETRIC Summary of this function goes here
%   Detailed explanation goes here

T = vec2tran(tform);
if(invert)
    T = inv(T);
end
T = gpuArray(single(T));

ims = size(images{1});

error = zeros(size(scans,1),1);
for i = 1:size(scans,1)
    [pro1, valid1] = projectLidar(T, K, scans{i}(:,1:3), ims(1:2));
    [pro2, valid2] = projectLidar(T, K, scans{i}(:,4:6), ims(1:2));

    
    valid = and(valid1,valid2);
    pro1c = pro1(valid,:);
    pro2c = pro2(valid,:);
    bootV = boot(valid);
    
    pro1 = interpolateImageUint8(images{i,1}, pro1c(:,1:2));
    pro2 = interpolateImageUint8(images{i,2}, pro2c(:,1:2));
    
    valid = and(pro1>0,pro2>0);
    
    err = ((pro1(:)-pro2(:)).^2);
    err = err(valid);
    bootV = bootV(valid);
    err = bootV.*err./mean(bootV);
    err = mean(err(isfinite(err)));
    
    error(i) = gather(err);
    if(isnan(error(i)))
        error(i) = 255;
    end
end

error = sqrt(mean(error(:),1));
