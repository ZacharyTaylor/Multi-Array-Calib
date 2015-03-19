function [ val, var ] = IndVar(offset, func, varargin )
%INDVAR Summary of this function goes here
%   Detailed explanation goes here

base = varargin(1:2:end);
baseV = varargin(2:2:end);
val = func(base{:});

var = zeros(size(val));
for i = 1:length(base)
    for j = 1:length(base{i})
        base{i}(j) = base{i}(j) + offset;
        temp = func(base{:});
        base{i}(j) = base{i}(j) - offset;
        
        temp = baseV{i}(j).*((temp-val)./offset).^2;
        var = var + temp;
    end
end

end

