function [ t ] = ReadTimeData( fileName )
%READTIMEDATA Reads the timestamp data for the ford and shrimp dataset
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   fileName- path of the time filename
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   t- nx1 time
%
%--------------------------------------------------------------------------
%   References:
%--------------------------------------------------------------------------
%   This function is part of the Multi-Array-Calib toolbox 
%   https://github.com/ZacharyTaylor/Multi-Array-Calib
%   
%   This code was written by Zachary Taylor
%   zacharyjeremytaylor@gmail.com
%   http://www.zjtaylor.com

%check input
validateattributes(fileName,{'char'},{'vector'});

if(~exist(fileName,'file'))
    error('%s is not a valid file');
end

fid = fopen(fileName,'r');

t = fread(fid, inf, '*uint64');

fclose(fid);

end

