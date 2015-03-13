function [ t,vt ] = ts2t( ts, vts )
%TS2T Summary of this function goes here
%   Detailed explanation goes here

t = repmat(ts(:,4),1,3).*ts(:,1:3);
vt = repmat(vts(:,4),1,3).*vts(:,1:3) + repmat(ts(:,4).^2,1,3).*vts(:,1:3) + repmat(vts(:,4),1,3).*(ts(:,1:3).^2);

end

