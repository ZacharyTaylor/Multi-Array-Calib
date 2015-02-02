function [ sensorData ] = loadSensorData( dataset, sensor, varargin )
%LOADSENSORDATA Automates the loading of stored sensor data for testing 
%   their performance
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   dataset- mx1 char array holding the name of the dataset to load
%   sensor- nx1 char array holding the name of the 1st sensor to load from
%       the dataset
%
%--------------------------------------------------------------------------
%   Optional Inputs:
%--------------------------------------------------------------------------
%   sensor2- px1 char array holding the name of the 2nd sensor to load from
%       the dataset
%   sensorn- qx1 char array holding the name of the nth sensor to load from
%       the dataset (no limit to the number of sensor inputs of this form)
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   sensorData- mx1 cell containing the sensor information
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
validateattributes(dataset,{'char'},{'vector'});
sensors = {sensor,varargin{:}};
for i = 1:length(sensors)
    validateattributes(sensors{i},{'char'},{'vector'});
end

%convert to lowercase first letter capitalized
dataset = lower(dataset);
dataset(1) = upper(dataset(1));
for i = 1:length(sensors)
    sensors{i} = lower(sensors{i});
    sensors{i}(1) = upper(sensors{i}(1));
end

%read in sensor data
sensorData = cell(length(sensors),1);
for i = 1:length(sensors)
    in = load(['./StoredTforms/' dataset sensors{i} 'Data.mat']);
    name = fieldnames(in);
    sensorData{i} = in.(name{1});
end

end

