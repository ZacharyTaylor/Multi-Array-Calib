load('Test_5_Res.mat');
load('Test_5_Res2.mat');

Res = abs(Res - repmat(mean(Res,2),1,size(Res,2)));
Res = reshape(Res,size(Res,1)*size(Res,2),size(Res,3));

Res = Res/1000000;

mRes = mean(Res,1);
vRes = std(Res,1);

Res2 = abs(Res2 - repmat(mean(Res2,2),1,size(Res2,2)));
Res2 = reshape(Res2,size(Res2,1)*size(Res2,2),size(Res2,3));

Res2 = Res2/1000000;

mRes2 = mean(Res2,1);
vRes2 = std(Res2,1);

load('Test_6_Res.mat');
load('Test_6_Res2.mat');

Res = abs(Res - repmat(mean(Res,2),1,size(Res,2)));
Res = reshape(Res,size(Res,1)*size(Res,2),size(Res,3));

Res = Res/1000000;

mRes3 = mean(Res,1);
vRes3 = std(Res,1);

Res2 = abs(Res2 - repmat(mean(Res2,2),1,size(Res2,2)));
Res2 = reshape(Res2,size(Res2,1)*size(Res2,2),size(Res2,3));

Res2 = Res2/1000000;

mRes4 = mean(Res2,1);
vRes4 = std(Res2,1);

load('Test_9_Res.mat');
load('Test_9_Res2.mat');

Res = abs(Res - repmat(mean(Res,2),1,size(Res,2)));
Res = reshape(Res,size(Res,1)*size(Res,2),size(Res,3));

Res = Res/1000000;

mRes5 = mean(Res,1);
vRes5 = std(Res,1);

Res2 = abs(Res2 - repmat(mean(Res2,2),1,size(Res2,2)));
Res2 = reshape(Res2,size(Res2,1)*size(Res2,2),size(Res2,3));

Res2 = Res2/1000000;

mRes6 = mean(Res2,1);
vRes6 = std(Res2,1);

x = (200:20:2000)';

addpath('./plotBounds');

%plot
close all
figure

%boundedline(x,mRes,vRes,'o-r');

subplot(1,2,2);
%semilogy(x,mRes,'x-b');
semilogy(x,mRes3,'o-r');
hold on;
semilogy(x,mRes5,'x-b');


title('10 seconds random offset');
set(gca,'layer','top');
set(gcf,'color','w');
legend('Shrimp','Ford')
axis([0 2000 0.0001 10])

subplot(1,2,1);
%semilogy(x,mRes2,'x-b');
semilogy(x,mRes4,'o-r');
hold on;
semilogy(x,mRes6,'x-b');
ylabel('Time error (s)');
xlabel('Frames used');
title('1 second random offset');
set(gca,'layer','top');
set(gcf,'color','w');
axis([0 2000 0.0001 10])




