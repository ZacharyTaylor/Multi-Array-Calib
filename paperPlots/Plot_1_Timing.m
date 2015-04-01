
data = cell(4,1);

%load the data
data{1} = load('../results/Test_1.1_Kitti_Time_1s.mat');
data{2} = load('../results/Test_1.2_Kitti_Time_5s.mat');
data{3} = load('../results/Test_1.3_Shrimp_Time_1s.mat');
data{4} = load('../results/Test_1.4_Shrimp_Time_5s.mat');
data{5} = load('../results/Test_1.5_Ford_Time_1s.mat');
data{6} = load('../results/Test_1.6_Ford_Time_5s.mat');

%average data
for i = 1:length(data)
    data{i} = data{i}.results;
    data{i}.Error = mean(abs(data{i}.Error),1);
    data{i}.Var = var(data{i}.Error);
    data{i}.Error = mean(data{i}.Error,2);
    data{i}.Var = mean(data{i}.Var,2);
    data{i}.Error = data{i}.Error(:);
    data{i}.Var = data{i}.Var(:);
end

%extract relevent data
x = repmat(data{1}.TimeLength',1,6);
y = zeros(size(x));
sd = zeros(size(x));
for i = 1:length(data);
    y(:,i) = data{i}.Error;
    sd(:,i) = sqrt(data{i}.Var);
end

figure;
hold on;
cmap = hsv(length(data));
for i = 1:length(data)
    boundedline(x(:,i),y(:,i),sd(:,i),'alpha','cmap',cmap(i,:));
end
set(gca,'XScale','log');
