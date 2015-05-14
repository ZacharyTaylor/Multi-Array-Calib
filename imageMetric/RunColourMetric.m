function [ error ] = RunColourMetric( tform, K, scans, images, updateRate, avVals )
%RUNFLOWMETRIC Summary of this function goes here
%   Detailed explanation goes here

persistent tnow;
persistent run;
if(isempty(tnow))
    tnow = now;
    run = 0;
end

T = inv(V2T(tform'));
T = gpuArray(single(T));

ims = size(images{1});

error = zeros(size(scans,1),1);

n = 0;
for i = 1:size(scans,1)
    [pro1, valid1] = projectLidar(T, K, scans{i}(:,1:3), ims(1:2));
    [pro2, valid2] = projectLidar(T, K, scans{i}(:,4:6), ims(1:2));
    
    dist = sqrt(sum(scans{i}(:,1:3).^2,2));
    
    valid = and(valid1,valid2);
    pro1c = pro1(valid,:);
    pro2c = pro2(valid,:);
    
    dist = dist(valid);
    
    pro1 = interpolateImage(images{i,1}, pro1c(:,1:2));
    pro2 = interpolateImage(images{i,2}, pro2c(:,1:2));
    
    valid = and(pro1>0,pro2>0);
    
    pro1 = single(pro1(valid,:));
    pro2 = single(pro2(valid,:));
    
    dist = dist(valid);
    
    err = dist.*abs((pro1-pro2));%./(pro1+pro2);
    
    error(i) = gather(sum(err));
    n = n + length(err);
end

error = sum(error);
if avVals
    error = error/n;
else
    %error = error*size(scans,1)/n;
end

if(isnan(error))
    error = 100;
end

run = run+1;
i = 1;
if((now - tnow) > updateRate/(3600*24))
    
    %display image
    disp1 = points2Image( gather([scans{i}(:,1:3),scans{i}(:,7:9)]), ims(1:2), gather(K), gather(T), 3, 0.3, true, repmat(double(gather(images{i,1}))/255,1,1,3) );
    disp1 = imresize(disp1, 1000/max(size(disp1)));
    disp2 = points2Image( gather([scans{i}(:,4:6),scans{i}(:,7:9)]), ims(1:2), gather(K), gather(T), 3, 0.3, true, repmat(double(gather(images{i,2}))/255,1,1,3) );
    disp2 = imresize(disp2, 1000/max(size(disp2)));
    
    if(size(disp1,1) > size(disp1,2))
        disp = [disp1,disp2];
    else
        disp = [disp1;disp2];
    end
    %imwrite(disp,['./gen/' num2str(run,'%08d'),'.jpg']);
    imshow(disp);
    
    t = gather(T);
    [r1,r2,r3] = dcm2angle(t(1:3,1:3)); t = [180*[r1,r2,r3]/pi,t(1,4),t(2,4),t(3,4)];
    text = sprintf('R: %2.2f P: %2.2f, Y: %2.2f, X: %1.2f, Y: %1.2f, Z: %1.2f, Err: %2.3f, Run: %i\n',t(1),t(2),t(3),t(4),t(5),t(6),error,run);
    xlabel(text);
    
    %set(gcf,'units','normalized','outerposition',[0.25 0.25 0.5 0.5])

    drawnow;
    tnow = now;
end

