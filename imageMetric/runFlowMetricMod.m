function [ error ] = runFlowMetricMod( tform, K, scans, images, invert )
%RUNFLOWMETRIC Summary of this function goes here
%   Detailed explanation goes here

% persistent tnow;
% if(isempty(tnow))
%     tnow = now;
% end

T = vec2tran(tform);
if(invert)
    T = inv(T);
end
T = gpuArray(single(T));

ims = size(images{1});

error = zeros(size(images,1),1);
for i = 1:size(images,1)
    [pro1, valid1] = projectLidar(T, K, [scans{i}(:,1:3),scans{i}(:,7)], ims(1:2));
    [pro2, valid2] = projectLidar(T, K, scans{i}(:,4:6), ims(1:2));

    valid = and(valid1,valid2);
    locs = pro1(valid,:);
    pro2 = pro2(valid,:);

    pro1 = interpolateImage(images{i}(:,:,1), locs(:,1:2));
    proLev = interpolateImage(images{i}(:,:,3), locs(:,1:2));
    pro2 = interpolateImage(images{i}(:,:,2), pro2(:,1:2));

    %err = -mean(proLev.*locs(:,3))./mean((pro1(:)-pro2(:)).^2);
    err = -sum(proLev.*locs(:,3))./mean(abs(pro1(:)-pro2(:)));
    
    error(i) = gather(err);
    if(isnan(error(i)))
        error(i) = 0;
    end
end

error = mean(error(:),1);

% i = 1;
% if((now - tnow) > 5/(3600*24))
%     %display image
%     Im1 = MyHistEq(gather(imageFromLidar( T, K, [scans{i}(:,1:3),scans{i}(:,7)], ims(1:2), 2 )));
%     Im1(Im1~=0) = 1- Im1(Im1~=0);
%     Im2 = MyHistEq(gather(images{i}(:,:,1)));
%     imshowpair(Im1,Im2,'ColorChannels','red-cyan');
% 
%     t = gather(T);
%     [r1,r2,r3] = dcm2angle(t(1:3,1:3)); t = [180*[r1,r2,r3]/pi,t(1,4),t(2,4),t(3,4)];
%     fprintf('R: %f P: %f, Y: %f, X: %f, Y: %f, Z: %f, Err: %f\n',t(1),t(2),t(3),t(4),t(5),t(6),error);
%     tnow = now;
%     drawnow;
% end

