%Plots the velocity of the different vechiles over their datasets

%load the data
CalibPath(true);
f = LoadSensorData('Ford','Nav');
k = LoadSensorData('Kitti','Nav');
s = LoadSensorData('Shrimp','Nav');

%get timing info
ft = (f{1}.time- min(f{1}.time))/1000000;
kt = double(k{1}.time- min(k{1}.time))/1000000;
st = double(s{1}.time- min(s{1}.time))/1000000;

%get time difference
fD = repmat([0.01;diff(ft)],1,6);
kD = repmat([0.01;diff(kt)],1,6);
sD = repmat([0.01;diff(st)],1,6);

%get angular and linear velocity
f = (f{1}.T_Skm1_Sk)./fD;
k = (k{1}.T_Skm1_Sk)./kD;
s = (s{1}.T_Skm1_Sk)./sD;

%median filter so it graphs nicely
f = medfilt1(f,100);
k = medfilt1(k,100);
s = medfilt1(s,100);

%% graph results
subplot(3,2,1);
hold on;
plot(ft,f(:,1),'r-');
plot(kt,k(:,1),'b-');
plot(st,s(:,1),'g-');

subplot(3,2,3);
hold on;
plot(ft,f(:,2),'r-');
plot(kt,k(:,2),'b-');
plot(st,s(:,2),'g-');

subplot(3,2,5);
hold on;
plot(ft,f(:,3),'r-');
plot(kt,k(:,3),'b-');
plot(st,s(:,3),'g-');

subplot(3,2,2);
hold on;
plot(ft,f(:,4),'r-');
plot(kt,k(:,4),'b-');
plot(st,s(:,4),'g-');

subplot(3,2,4);
hold on;
plot(ft,f(:,5),'r-');
plot(kt,k(:,5),'b-');
plot(st,s(:,5),'g-');

subplot(3,2,6);
hold on;
plot(ft,f(:,6),'r-');
plot(kt,k(:,6),'b-');
plot(st,s(:,6),'g-');

