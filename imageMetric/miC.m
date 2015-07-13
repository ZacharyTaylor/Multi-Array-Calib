function [ mi ] = miC( A, B, normal, bins )
% MIC A C implementation to calculate the mutual information
% between two matricies.
%
%   Required Inputs:
%   A- matrix with all points in the range 0 to 1
%   B- matrix the same size as A with all points in the range 0 to 1
%
%   Optional Inputs:
%   normal- true for normalized mutual information, false for standard
%      (default false)
%   bins- number of bins to use in calculating MI. 
%      (default 110, see performance notes for reason)
%
%   Outputs:
%   mi- mi of A and B
%
%   Performance Notes:
%   For a large speed gain pass A and B in as gpuArrays
%   Due to how GPUs divide work the runtime will decrease as bins increases
%   until (4*bins)^2 is larger then the shared memory, a which point the 
%   runtime will roughly scale by (ceil((4*bins)^2/(shared memory in bytes)). 
%   Note for most GPUs the shared memory is 48kB resulting in the optimal bin
%   number from a runtime point of view being 110.
%   The performance will also increase the closer to equally distributed
%   the histograms are.
%
%   References-
%   This code was used in generating the results for the journal paper
%   Multi-modal sensor calibration using a gradient orientation measure 
%   http://www.zjtaylor.com/welcome/download_pdf?pdf=JFR2013.pdf as well
%   as several of my other publications
%
%   This code was written by Zachary Taylor
%   zacharyjeremytaylor@gmail.com
%   http://www.zjtaylor.com

%% check inputs
if(isnumeric(A))
    A = single(A);
else
    error('A must be numeric');
end

if(isnumeric(B))
    B = single(B);
else
    error('B must be numeric');
end

if(numel(A) ~= numel(B))
    error('A and B must contain the same number of elements');
end

if(nargin < 3)
    normal = [];
end
if(isempty(normal))
    normal = false;
else
    validateattributes(normal, {'logical'},{'scalar'});
end

if(nargin < 4)
    bins = [];
end
if(isempty(bins))
    bins = 255;
else
    validateattributes(bins, {'numeric'},{'scalar','integer','positive'});
end

mi = miCMex(A, B, normal, uint32(bins));

end

