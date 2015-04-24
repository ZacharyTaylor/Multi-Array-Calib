data = {};

%load the data
data{1} = load('../storedTforms/KittiCam1Data.mat');
data{2} = load('../storedTforms/ShrimpCam1Data.mat');
data{3} = load('../storedTforms/FordCam1Data.mat');

y = cell(size(data));
x = y;
for i = 1:length(data)
    y{i} = sqrt(sum(data{i}.cam1Data.T_Skm1_Sk(:,4:6).^2,2));
    x{i} = double(data{i}.cam1Data.time);
    x{i} = (x{i} - x{i}(1))/1000000;
    y{i} = y{i}./median(diff(x{i}));
    y{i}(y{i} > 1) = 0;
    
    %y{i} = [0;diff(y{i})];
    y{i} = abs(y{i});
end

subplot(3,1,1);
plot(x{1},y{1});
axis([0 600 -0.1 0.8])
title('Kitti dataset');
subplot(3,1,2);
plot(x{2},y{2});
axis([0 600 -0.1 0.8])
title('Shrimp dataset');
ylabel('Rotational Speed (rad/s)');
subplot(3,1,3);
plot(x{3},y{3});
axis([0 600 -0.1 0.8])
title('Ford dataset');
xlabel('Time (s)');