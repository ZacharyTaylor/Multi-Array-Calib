function [ val, var ] = IndVarVec(offset, func, varargin )
%INDVAR Summary of this function goes here
%   Detailed explanation goes here

base = varargin(1:2:end);
baseV = varargin(2:2:end);
val = func(varargin{:});

var = zeros(size(val));
for i = 1:length(base)
    for j = 1:size(base{i},2)
        for k = 1:size(base{i},3)
            si = ones(size(base{i}(:,j,k)));
            %si(1:floor(length(si)/2))=-1;
            base{i}(:,j,k) = base{i}(:,j,k) + si.*sqrt(baseV{i}(:,j,k)).*offset;
            joint = cell(size(varargin));
            joint(1:2:end) = base;
            joint(2:2:end) = baseV;
            temp = func(joint{:});
            base{i}(:,j,k) = base{i}(:,j,k) - si.*sqrt(baseV{i}(:,j,k)).*offset;

            temp = ((temp-val)./offset).^2;
            var = var + temp;
        end
    end
end

end

