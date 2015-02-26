function [epoch, epochStart, epochEnd] = ReadKittiTimestamps( path )
%READKITTITIMESTAMPS reads in the timestamp kitti file and converts to 
%   microseconds since 1970
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   path- path to where the timestamps are stored
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   epoch, nx1 list of epoch timestamps when the images were taken
%   epochStart, first timestamp
%   epochEnd, last timestamp
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

validateattributes(path,{'char'},{'vector'});
if(~exist(path,'dir'))
    error('%s is not a valid directory');
end

fid = fopen([path 'timestamps.txt'],'r');

time = textscan(fid,'%d-%d-%d %d:%d:%f');
year = time{1};
month = time{2};
day = time{3};
hour = time{4};
minute = time{5};
second = time{6};

E=repmat([1970 01 01 00 00 00],size(year,1),1);
D = double([year, month, day, hour, minute, floor(second)]);
epoch = uint64((datenum(D)-datenum(E))*86400000000 + 1000000*(second-floor(second)));

fclose(fid);

fid = fopen([path 'timestamps_start.txt'],'r');
if(fid == -1)
    epochStart = epoch;
else
    time = textscan(fid,'%d-%d-%d %d:%d:%f');
    year = time{1};
    month = time{2};
    day = time{3};
    hour = time{4};
    minute = time{5};
    second = time{6};

    E=repmat([1970 01 01 00 00 00],size(year,1),1);
    D = double([year, month, day, hour, minute, floor(second)]);
    epochStart = uint64((datenum(D)-datenum(E))*86400000000 + 1000000*(second-floor(second)));

    fclose(fid);
end

fid = fopen([path 'timestamps_end.txt'],'r');
if(fid == -1)
    epochEnd = epoch;
else
    time = textscan(fid,'%d-%d-%d %d:%d:%f');
    year = time{1};
    month = time{2};
    day = time{3};
    hour = time{4};
    minute = time{5};
    second = time{6};

    E=repmat([1970 01 01 00 00 00],size(year,1),1);
    D = double([year, month, day, hour, minute, floor(second)]);
    epochEnd = uint64((datenum(D)-datenum(E))*86400000000 + 1000000*(second-floor(second)));

    fclose(fid);
end

end

