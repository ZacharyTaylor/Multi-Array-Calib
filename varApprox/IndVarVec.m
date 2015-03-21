function [ val, var ] = IndVarVec(offset, func, varargin )
%INDVAR Summary of this function goes here
%   Detailed explanation goes here

base = varargin(1:2:end);
baseV = varargin(2:2:end);
val = func(varargin{:});

var = zeros(size(val));
for i = 1:length(base)
    for j = 1:size(base{i},2)
        base{i}(:,j) = base{i}(:,j) + sqrt(baseV{i}(:,j)).*offset;
        joint = cell(size(varargin));
        joint(1:2:end) = base;
        joint(2:2:end) = baseV;
        temp = func(joint{:});
        base{i}(:,j) = base{i}(:,j) - sqrt(baseV{i}(:,j)).*offset;
        
        temp = ((temp-val)./offset).^2;
        var = var + temp;
    end
end

end

