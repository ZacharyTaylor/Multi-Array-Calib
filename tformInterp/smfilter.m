function [ out ] = smfilter( in, percent )
%SMFILTER Summary of this function goes here
%   Detailed explanation goes here

in = in(:);
wind = gausswin(ceil(percent*length(in)/100));
wind = wind./sum(wind);
out = conv([repmat(in(1),ceil(percent*length(in)/2),1);in;repmat(in(end),ceil(percent*length(in)/2),1)],wind,'same');
out = out(ceil(percent*length(in)/2)+1:end-ceil(percent*length(in)/2));


end

