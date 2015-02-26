function [ image ] = Undistort(imageD, D, K)
%FORDCAMINFO Sets the directory layout, masks and intrinsics for the ford
%   dataset
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   imageD- distorted input image
%   D- 5x1 distortion vector
%   K-3x4 camera matrix
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   I- undistorted image
%
%--------------------------------------------------------------------------
%   References:
%--------------------------------------------------------------------------
%   This function is a slightly modified version of this stackoverflow code
%   http://stackoverflow.com/questions/12117825/how-can-i-undistort-an-image-in-matlab-using-the-known-camera-parameters

validateattributes(imageD,{'numeric'},{'2d'});
validateattributes(D,{'numeric'},{'size',[5,1]});
validateattributes(K,{'numeric'},{'size',[3,4]});


fx = K(1,1);
fy = K(2,2);
cx = K(1,3);
cy = K(2,3);
k1 = D(1);
k2 = D(2);
k3 = D(5);
p1 = D(3);
p2 = D(4);

image = zeros(size(imageD));
[i, j] = find(~isnan(image));

% Xp = the xyz vals of points on the z plane
Xp = K\[j i ones(length(i),1)]';

% Now we calculate how those points distort i.e forward map them through the distortion
r2 = Xp(1,:).^2+Xp(2,:).^2;
x = Xp(1,:);
y = Xp(2,:);

x = x.*(1+k1*r2 + k2*r2.^2 + k3*r2.^3) + 2*p1.*x.*y + p2*(r2 + 2*x.^2);
y = y.*(1+k1*r2 + k2*r2.^2 + k3*r2.^3) + 2*p2.*x.*y + p1*(r2 + 2*y.^2);

% u and v are now the distorted cooridnates
u = reshape(fx*x + cx,size(image));
v = reshape(fy*y + cy,size(image));

c = class(imageD);

% Now we perform a backward mapping in order to undistort the warped image coordinates
image = interp2(double(imageD), u, v);

image = eval([c '(image)']);

end

