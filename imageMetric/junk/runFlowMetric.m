function [ error ] = runFlowMetric( tform, K, scans, images, invert )
%RUNFLOWMETRIC Summary of this function goes here
%   Detailed explanation goes here

persistent tnow;
persistent run;
if(isempty(tnow))
    tnow = now;
    run = 0;
end

T = V2T(tform');
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

    pro1 = interpolateImageUint8(images{i,1}, pro1c(:,1:2));
    pro2 = interpolateImageUint8(images{i,2}, pro2c(:,1:2));
    
    valid = and(pro1>0,pro2>0);
    
    err = (pro1(valid)-pro2(valid)).^2;
    
    %err = sort(err);
    %err = err(1:round(size(err,1)*0.9));
    
    err = median(err(isfinite(err)));
    
    error(i) = gather(err);
    if(isnan(error(i)))
        error(i) = 255;
    end
end

error = sqrt(mean(error(:),1));

run = run+1;
i = 1;
if((now - tnow) > 5/(3600*24))
    
    %display image
    disp = points2Image( gather([scans{i}(:,1:3),scans{i}(:,7:9)]), ims(1:2), gather(K), gather(T), 3, 0.3, true, repmat(double(gather(images{i,1}))/255,1,1,3) );
    
    imshow(disp);
    drawnow;
    
    t = gather(T);
    [r1,r2,r3] = dcm2angle(t(1:3,1:3)); t = [180*[r1,r2,r3]/pi,t(1,4),t(2,4),t(3,4)];
    fprintf('R: %f P: %f, Y: %f, X: %f, Y: %f, Z: %f, Err: %f, Run: %i\n',t(1),t(2),t(3),t(4),t(5),t(6),error,run);
    tnow = now;
end

