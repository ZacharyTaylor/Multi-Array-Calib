function [ err ] = findErr(offset,T,C,t,samples)
%FINDERR Summary of this function goes here
%   Detailed explanation goes here

offset = [0;offset'];

persistent a;
if(isempty(a))
    a = 0;
end

%add time offset
tMin = 0;
tMax = inf;
for i = 1:length(t)
    t{i} = t{i} + offset(i);
    tMin = max(tMin,t{i}(1));
    tMax = min(tMax,t{i}(end));
end

tI = tMin:(tMax-tMin)/(samples-1):tMax;

for i = 1:length(T)
    T{i} = abs(diff(interp1(t{i},T{i},tI)));
    C{i} = interp1(t{i},C{i},tI(1:end-1));
end

a = a+1;
if(a > 500)
    a = 0;
    cc=hsv(length(T));
    
    hold off;
    plot(T{1},'color',cc(1,:));
    hold on;
    
    for i = 2:length(T)       
        plot(T{i},'color',cc(i,:));
    end
    %offset
    drawnow;
end
    
err = 0;
for i = 1:length(T)
    for j = i+1:length(T)
        err = err + mean(((T{i} - T{j}).^2)./(C{i} + C{j}));
    end
end

end

