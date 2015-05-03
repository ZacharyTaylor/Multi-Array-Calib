function [ out, outV ] = varChange( a, va, gt )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

out = zeros(100,1);
for i = 1:100
    as = a + randn(size(a)).*va;
    [out(i,1),out(i,2),out(i,3)] = dcm2angle(V2R(as)/V2R(gt));
    out(i,:) = (out(i,:))*180/pi;
end

outV = std(out);

[r,p,y] = dcm2angle(V2R(a));
out = abs([r,p,y])*180/pi;

