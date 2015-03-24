function [ out, outV ] = CombEst( varargin )
%COMBEST Summary of this function goes here
%   Detailed explanation goes here

base = varargin(1:2:end);
baseV = varargin(2:2:end);

out = zeros(size(base{1}));
outV = out;

for i = 1:length(base)
    baseV{i} = 1./baseV{i};
    out = out + base{i}.*baseV{i};
    outV = outV + baseV{i};
end

out = out./outV;
outV = 1./outV;

end

