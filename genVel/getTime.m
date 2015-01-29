function [ time ] = getTime( pos3D, i )
%GETTIME Summary of this function goes here
%   Detailed explanation goes here

time =atan2(pos3D(:,2),-pos3D(:,1));
time(time < 0) = 2*pi+time(time < 0);
time = time/(2*pi);
time = time + i - 0.5;
end

