function [ out ] = matchInt( A, B, warp, samples )
%MATCHINT Summary of this function goes here
%   Detailed explanation goes here

persistent t;
if(isempty(t))
    t = 0;
end

scale = warp(1);
offset = warp(2);

%find overlapping regions
r = 0:1/samples:1;
rA = r;
rB = scale*r + offset;

valid = and(rB >= 0, rB <=1);
rA = (size(A,1)-1)*rA(valid)+1;
rB = (size(B,1)-1)*rB(valid)+1;

iA = interp1(1:size(A,1),A,rA,'pchip');
iB = interp1(1:size(B,1),B,rB,'pchip');

iA = sqrt(sum(iA.^2,2));
iB = sqrt(sum(iB.^2,2));

out = mean((iA-iB).^2);

if(isnan(out))
    out = inf;
end

t = t + 1;
if(t == 1000)
    fprintf('Error %3.3f Scale: %3.3f, Offset %3.3f\n',out,scale,offset);
    hold off;
    plot(iA,'b-');
    hold on;
    plot(iB,'r-');
    drawnow;
    t = 0;
end

end

