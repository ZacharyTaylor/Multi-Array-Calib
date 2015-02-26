function [ var ] = EstPointVar( image, points )
%ESTPOINTVAR Summary of this function goes here
%   Detailed explanation goes here

%sobel opertors
filtX = [-1 0 1;-2 0 2; -1 0 1];
filtY = [-1 -2 -1; 0 0 0; 1 2 1];

%image coordinates
locX = repmat(points(:,1),1,9);
locX = locX + repmat([-1,-1,-1,0,0,0,1,1,1],size(points,1),1);
locY = repmat(points(:,2),1,9);
locY = locY + repmat([-1,0,1,-1,0,1,-1,0,1],size(points,1),1);

%original positions
[X,Y] = meshgrid(1:size(image,2),1:size(image,1));

%intensities
int = interp2(X,Y,image,locX,locY);

%gradients
dX = sum(filtX.*repmat(int(:)',size(points,1),1),2);
dY = sum(filtY.*repmat(int(:)',size(points,1),1),2);

%estimate variance
var = [dY.*dY, -dY.*dX; -dX.*dY, dX.*dX]./(dY.*dY.*dX.*dX);

end

