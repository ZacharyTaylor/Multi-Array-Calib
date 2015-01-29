function [ offset ] = findErr3(T,t,samples)
%FINDERR Summary of this function goes here
%   Detailed explanation goes here

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
    
    for j = size(T{i},1):-1:2
        T{i}(j,:) = R2V(V2R(T{i}(j,:))/V2R(T{i}(j-1,:)));
    end
    T{i} = T{i}(2:end,:);
    
    T{i} = sqrt(sum(T{i}.^2,2));
    T{i} = medfilt1(T{i},10);
    
    %T{i} = (T{i} - mean(T{i})) / std(T{i});
    %T{i} = smooth(T{i});
    %T{i} = medfilt1(T{i},10);
end

P = nchoosek(1:length(T),2);
v = zeros(size(P,1)+1,1);
x = zeros(size(P,1)+1,length(T));
for i = 1:size(P,1)
    temp = xcorr(T{P(i,1)},T{P(i,2)});
    [val,idx] = imax(temp);
    v(i) = val*(idx - samples);
    x(i,P(i,1)) = val;
    x(i,P(i,2)) = -val;
end

x(end,1) = 100000;

offset = ((tMax-tMin)/samples)*(x\v);

end

