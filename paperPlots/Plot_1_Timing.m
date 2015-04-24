data = {};

%load the data
data{1} = load('../results/Test_1.1_Kitti_Time_0.1s.mat');
data{2} = load('../results/Test_1.2_Kitti_Time_1s.mat');
data{3} = load('../results/Test_1.3_Kitti_Time_5s.mat');
data{4} = load('../results/Test_1.4_Shrimp_Time_0.1s.mat');
data{5} = load('../results/Test_1.5_Shrimp_Time_1s.mat');
data{6} = load('../results/Test_1.6_Shrimp_Time_5s.mat');
data{7} = load('../results/Test_1.7_Ford_Time_0.1s.mat');
data{8} = load('../results/Test_1.8_Ford_Time_1s.mat');
data{9} = load('../results/Test_1.9_Ford_Time_5s.mat');

%average data
y = cell(size(data));
for i = 1:length(data)
    data{i} = data{i}.results;
    data{i}.Error = mean(data{i}.Error(2:end,:,1:2:end),1);

    data{i}.Error = abs(data{i}.Error);
    y{i} = reshape(data{i}.Error,size(data{i}.Error,1)*size(data{i}.Error,2),[]);
end

x = data{1}.TimeLength(1:2:end);

h = figure;

subplot(3,1,1);
boxplot(y{7},x,'datalim',[0.0001,inf])
set(gca,'YScale','log')
title('0.1 Seconds Random Offset');
ax = gca;
ax.YTick = [0.0001,0.001,0.01,0.1,1,10];

subplot(3,1,2);
boxplot(y{8},x,'datalim',[0.0001,inf])
set(gca,'YScale','log')
title('1 Second Random Offset');
ylabel('Final Timing Offset (s)');
ax = gca;
ax.YTick = [0.0001,0.001,0.01,0.1,1,10];

subplot(3,1,3);
boxplot(y{9},x,'datalim',[0.0001,inf])
set(gca,'YScale','log')
title('5 Seconds Random Offset');
xlabel('Length of Sensor Data used (s)');
ax = gca;
ax.YTick = [0.0001,0.001,0.01,0.1,1,10];

set(h, 'Position', [100, 100, 707, 1000]);