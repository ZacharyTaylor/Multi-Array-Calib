function [ mat ] = CrossMat( vec )
%CROSSMAT Summary of this function goes here
%   Detailed explanation goes here

mat = [0,-vec(3),vec(2);vec(3),0,-vec(1);-vec(2),vec(1),0];

end

