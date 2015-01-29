function [ offset ] = findErr2(T,S,t,samples)
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
    T{i} = abs(diff(T{i}));
    
    S{i} = interp1(t{i},S{i},tI,'pchip');
    S{i} = 1./abs(diff(S{i}));

    T{i} = MyHistEq(T{i});
    T{i} = (T{i} - mean(T{i})) / std(T{i});
end

P = nchoosek(1:length(T),2);
v = zeros(size(P,1)+1,1);
x = zeros(size(P,1)+1,length(T));
for i = 1:size(P,1)
    temp = wncc(T{P(i,1)},T{P(i,2)},(S{P(i,1)}+S{P(i,2)}));
    [val,idx] = imax(temp);
    v(i) = val*(ceil(samples/2) - idx);
    x(i,P(i,1)) =val;
    x(i,P(i,2)) = -val;
end

x(end,1) = 100000;

offset = ((tMax-tMin)/samples)*(x\v);

end

