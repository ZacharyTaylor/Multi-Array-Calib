function [] = plotTimingAlignment()
%PLOTTIMINGALIGNMENT

tformIdx = 1;
samples = 10000;

load('shrimpCam1Data.mat');
sensorData{tformIdx,1} = cam1Data;
tformIdx = tformIdx + 1;

load('shrimpVelData.mat');
sensorData{tformIdx,1} = velData;

T = cell(length(sensorData),1);
t = cell(length(sensorData),1);
for i = 1:length(sensorData)
    [~,idx] = sort(sensorData{i}.time);
    valid = diff(double(sensorData{i}.time(idx))) > 0;
    idx = idx(valid);
    t{i} = double(sensorData{i}.time(idx));
    
    %get absolute angle magnitude
    T{i} = sqrt(sum(sensorData{i}.T_S1_Sk(idx,4:6).^2,2));
end

t{1} = t{1} + 2000000;

idx = [t{1}(2200),t{1}(2400)]';
%get scans
for j = 1:length(T)
    valid = and(t{j} <= idx(2), t{j} >= idx(1));
    T{j} = T{j}(valid);
    t{j} = t{j}(valid);
end

close all;
plot(t{1},T{1}); hold on; plot(t{2},T{2},'r-');

%add time offset
tMin = 0;
tMax = inf;
for i = 1:length(t)
    tMin = max(tMin,t{i}(1));
    tMax = min(tMax,t{i}(end));
end

tI = tMin:(tMax-tMin)/(samples):tMax;

for i = 1:length(T)    
    T{i} = interp1(t{i},T{i},tI,'pchip');
    %T{i} = abs(diff(T{i}));
    %T{i} = MyHistEq(T{i});
end

figure;
plot(tI,T{1}); hold on; plot(tI,T{2},'r-');

tI = tI(2:end);

for i = 1:length(T)    
    T{i} = abs(diff(T{i}));
end

figure;
plot(tI,T{1}); hold on; plot(tI,T{2},'r-');

for i = 1:length(T)    
    T{i} = MyHistEq(T{i});
end

figure;
plot(tI,T{1}); hold on; plot(tI,T{2},'r-');

temp = xcorr(T{1},T{2});
figure;
plot(temp);

end

