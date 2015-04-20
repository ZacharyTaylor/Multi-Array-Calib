data = {};

%load the data
%data{1} = load('../results/Test_1.1_Kitti_Time_1s.mat');
%data{2} = load('../results/Test_1.2_Kitti_Time_5s.mat');
data{1} = load('../results/Test_1.3_Shrimp_Time_1s.mat');
%data{4} = load('../results/Test_1.4_Shrimp_Time_5s.mat');
%data{5} = load('../results/Test_1.5_Ford_Time_1s.mat');
%data{6} = load('../results/Test_1.6_Ford_Time_5s.mat');

%average data
for i = 1:length(data)
    data{i} = data{i}.results;
    temp = zeros(size(data{i}.Error,3),1);
    tempV = zeros(size(data{i}.Error,3),1);
    for j = 1:size(data{i}.Error,3)
        in = data{i}.Error(:,:,j); in = in(:);
        temp(j) = mean(abs(in(isfinite(in))));
        tempV(j) = var(abs(in(isfinite(in))));
    end
    data{i}.Error = temp;
    data{i}.Var = tempV;
end

%extract relevent data
x = repmat(data{1}.TimeLength',1,length(data));
y = zeros(size(x));
sd = zeros(size(x));
for i = 1:length(data);
    y(:,i) = data{i}.Error;
    sd(:,i) = sqrt(data{i}.Var);
end

%get colours
h = plot(ones(10,10));
c = get(h,'Color');

sdup = y + sd;
sddown = max(y-sd,0);

plot(x(:,1),y(:,1),'Color',c{1})
hold on
set(gca,'YScale','log');
axis manual

xlabel('Length of sensor observations (s)');
ylabel('Mean timing offset error (s)');

cb = brighten(c{1},0.7);
area(x(:,1),sdup(:,1),'EdgeColor','none','FaceColor',cb)
area(x(:,1),sddown(:,1),'EdgeColor','none','FaceColor',[1,1,1])
plot(x(:,1),y(:,1),'Color',c{1})