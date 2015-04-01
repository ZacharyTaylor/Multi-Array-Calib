function [] = trimmedTest()

%set seed for repeatability
rng('default');
rng(1);

data = [randn(450,2);randn(50,2)+repmat([-5,5],50,1)];

steps = 50;

m = zeros(steps);
med = zeros(steps);
trim = zeros(steps);
thres = zeros(steps);

for x = 1:steps
    x
    for y = 1:steps
        p = 2*[x/steps-0.5,y/steps-0.5];
        m(x,y) = offset(p,data,'mean');
        med(x,y) = offset(p,data,'median');
        trim(x,y) = offset(p,data,'trim');
        thres(x,y) = offset(p,data,'thres');
        
    end
end

pM = pso(@(p) offset(p,data,'mean'), 2,[],[],[],[],[-1,-1],[1,1]);
pMed = pso(@(p) offset(p,data,'median'), 2,[],[],[],[],[-1,-1],[1,1]);
pTrim = pso(@(p) offset(p,data,'trim'), 2,[],[],[],[],[-1,-1],[1,1]);
pStu = pso(@(p) offset(p,data,'studentt'), 2,[],[],[],[],[-1,-1],[1,1]);
pThes = pso(@(p) offset(p,data,'thres'), 2,[],[],[],[],[-1,-1],[1,1]);

%plot everything
plot(data(:,1),data(:,2),'r.');
hold on;
plot(0,0,'o','MarkerSize',10,'MarkerFaceColor','w','MarkerEdgeColor','k');
plot(pM(1),pM(2),'d','MarkerSize',8,'MarkerFaceColor','c','MarkerEdgeColor','k');
plot(pMed(1),pMed(2),'s','MarkerSize',8,'MarkerFaceColor','g','MarkerEdgeColor','k');
plot(pTrim(1),pTrim(2),'p','MarkerSize',8,'MarkerFaceColor','b','MarkerEdgeColor','k');
plot(pThes(1),pThes(2),'h','MarkerSize',8,'MarkerFaceColor','y','MarkerEdgeColor','k');
plot(pStu(1),pStu(2),'o','MarkerSize',8,'MarkerFaceColor','m','MarkerEdgeColor','k');

legend('Data','True Centre','Mean', 'Median', 'Trimmed Mean', 'Threshold', '0.1 Nrom');

figure;
[x,y] = meshgrid(2*((1:steps)/steps-0.5),2*((1:steps)/steps-0.5));
subplot(1,4,1);
surf(x,y,m);
view(135.5,12);

subplot(1,4,2);
surf(x,y,med);
view(135.5,12);

subplot(1,4,3);
surf(x,y,trim);
view(135.5,12);

subplot(1,4,4);
surf(x,y,thres);
view(135.5,12);

end

function [err] = offset(p,data,metric)
    dist = repmat(p,size(data,1),1) - data;
    dist = (sum(dist.^2,2));
    
    if(strcmpi(metric,'mean'))
        err = sqrt(mean(dist));
    elseif(strcmpi(metric,'median'))
        err = sqrt(median(dist));
    elseif(strcmpi(metric,'trim'))
        temp = sort(dist,'ascend');
        temp = temp(1:floor(size(temp,1)*0.5));
        err = sqrt(mean(temp));
    elseif(strcmpi(metric,'studentt'))
        err = sqrt(mean(dist.^0.1));
    else
        temp = dist(dist < 9);
        err = sqrt(mean(temp));
    end
end
