function [ var ] = EstPointVar( image, points )
%ESTPOINTVAR Summary of this function goes here
%   Detailed explanation goes here

[dX,dY] = imgradientxy(image);

%original positions
[X,Y] = meshgrid(1:size(image,2),1:size(image,1));

dX = interp2(X,Y,dX,points(:,1),points(:,2));
dY = interp2(X,Y,dY,points(:,1),points(:,2));

%estimate variance
var = [dY.*dY, -dY.*dX, -dX.*dY, dX.*dX]./repmat((dY.*dY.*dX.*dX),1,4);
var(isnan(var)) = inf;

end

