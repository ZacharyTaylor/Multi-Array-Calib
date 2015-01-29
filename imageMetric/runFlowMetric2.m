function [ error ] = runFlowMetric2( tform, K, scans, images, invert )
%RUNFLOWMETRIC Summary of this function goes here
%   Detailed explanation goes here

persistent tnow;
if(isempty(tnow))
    tnow = now;
end

T = vec2tran(tform);
if(invert)
    T = inv(T);
end
T = gpuArray(single(T));

ims = size(images{end});

error = zeros(size(scans,1),1);
for i = 1:size(scans,1)
    [pro1, valid1] = projectLidar(T, K, scans{i}(:,1:3), ims(1:2));
    [pro2, valid2] = projectLidar(T, K, scans{i}(:,4:7), ims(1:2));

    valid = and(valid1,valid2);
    pro1 = pro1(valid,:);
    pro2 = pro2(valid,:);

    pro1 = gather(pro1(:,1:2));
    pro2 = gather(pro2(:,1:2));

    [idx,d] = knnsearch(pro1,images{i}(:,1:2));
    
    valid = d < 3;
    idx = idx(valid);
    
    error(i) = mean(sqrt(sum((pro2(idx,:) - images{i}(valid,3:4)).^2,2)));
    
    if(isnan(error(i)))
        error(i) = 255;
    end
end

error = mean(error(:),1);

if((now - tnow) > 3/(3600*24))
    %display image
    disp = points2Image( gather([scans{1}(:,1:3),scans{1}(:,7:9)]), ims(1:2), gather(K), gather(T), 3, 0.4, true, repmat(double((images{end,1}))/255,1,1,3));
    sum(valid)
    imshow(disp);
    drawnow;
    
    t = gather(T);
    [r1,r2,r3] = dcm2angle(t(1:3,1:3)); t = [180*[r1,r2,r3]/pi,t(1,4),t(2,4),t(3,4)];
    fprintf('R: %f P: %f, Y: %f, X: %f, Y: %f, Z: %f, Err: %f\n',t(1),t(2),t(3),t(4),t(5),t(6),error);
    tnow = now;
end

