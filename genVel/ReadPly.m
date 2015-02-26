function [ data ] = ReadPly( filename )
%READPLY Reads in dataset from plyfile (WARNING WILL NOT READ GENERAL PLY
%FILES, FOR THEM USE PLY_READ)
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   filename-location of ply file to read
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   data- data stored in ply
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

%open
[ fid, Msg ] = fopen (filename, 'r', 'ieee-be');

if ( fid == -1 )
    error(Msg);
end

lineSize = 19;

header = fread(fid,500,'*char')';

%get length of header
key = 'end_header';
idx = strfind(header, key);
header = header(1:(idx(1)+length(key)));

%get length of data
key = 'element vertex ';
idx = strfind(header, key);
numel = sscanf(header(idx(1) + length(key):end), '%g', 1);

%seek to start of data
fseek(fid,length(header),'bof');

%read data
data.vertex.x = fread(fid,numel,'float',lineSize-4);
fseek(fid,length(header)+4,'bof');
data.vertex.y = fread(fid,numel,'float',lineSize-4);
fseek(fid,length(header)+8,'bof');
data.vertex.z = fread(fid,numel,'float',lineSize-4);
fseek(fid,length(header)+12,'bof');

data.vertex.intensity = fread(fid,numel,'uchar',lineSize-1);
fseek(fid,length(header)+13,'bof');

data.vertex.timeOffset = fread(fid,numel,'int32',lineSize-4);
fseek(fid,length(header)+17,'bof');

data.vertex.beamId = fread(fid,numel,'uchar',lineSize-1);
fseek(fid,length(header)+18,'bof');

data.vertex.valid = fread(fid,numel,'uchar',lineSize-1);

fclose(fid);


end

