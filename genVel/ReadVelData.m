function [ xyz, intensity, timeFrac ] = ReadVelData( velFile )
%GENCAM Reads in velodyne file (accepts KITTI's .txt and .bins and Shrimps
%   .ply format
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   velFile- velodyne file path
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   xyz- mx3 list of velodynes 3d points
%   intensity- mx1 list of the intensity of the velodyne points
%   timeFrac- mx1 list of fractional timing data from velodyne
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

%check inputs
validateattributes(velFile,{'char'},{'vector'});

if(~exist(velFile,'file'))
    error('%s is not a valid file');
end

%get extension
ext = velFile(end-2:end);

%read in information
switch(lower(ext))
    case('ply')
        %read in ply data
        data = ReadPly(velFile);
        xyz = [data.vertex.x, data.vertex.y, data.vertex.z];
        intensity = data.vertex.intensity;
        time = data.vertex.timeOffset;
        valid = data.vertex.valid > 0;
        
        intensity = intensity./max(intensity);
        
        %get valid sections
        xyz = xyz(valid,:);
        intensity = intensity(valid);
        time = time(valid);
        
        %convert from time to timeFrac
        timeFrac = time/(max(time) - min(time));
        
    case('txt')
        %read in txt data
        data = dlmread(velFile,' ');
        
        %split up file
        xyz = data(:,1:3);
        intensity = data(:,4);
        
        %generate timing data (not provided)
        [timeFrac,valid] = GenVelTimeFrac([-xyz(:,1), xyz(:,2), -xyz(:,3)]);
        
        %throw away ends of scans (timing too unreliable)
        xyz = xyz(valid,:);
        intensity = intensity(valid);
        timeFrac = timeFrac(valid);

    case('bin')
        %read in binary data
        data = ReadKittiVelDataSingle(velFile);
        
        %split up file
        xyz = data(:,1:3);
        intensity = data(:,4);
        
        %generate timing data (not provided)
        [timeFrac,valid] = GenVelTimeFrac([-xyz(:,1), xyz(:,2), -xyz(:,3)]);
        
        %throw away ends of scans (timing too unreliable)
        xyz = xyz(valid,:);
        intensity = intensity(valid,:);
        timeFrac = timeFrac(valid);
        
    otherwise
        error('format not recognised');
end

end

