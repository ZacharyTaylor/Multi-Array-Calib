function [ sensorData, valid ] = RejectPoints( sensorData, maxSD, minMove )
%REJECTPOINTS Removes points the sensor data can't agree on and points with
%   little movement
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   sensorData- nx1 cell containing sensor data sturcts
%   maxSD- maximum number of standard diviations data can differ by
%   minMove- minimum rotational velocity of points
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   sensorData- nx1 cell containing sensor data sturcts
%   valid- mx1 index of acceptable points
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
validateattributes(sensorData,{'cell'},{'vector'});
validateattributes(maxSD,{'numeric'},{'scalar','positive'});
validateattributes(minMove,{'numeric'},{'scalar','positive'});

%preallocate memory
Mag = zeros(size(sensorData{1}.T_Skm1_Sk,1),size(sensorData,1));
Var = Mag;
times = Mag;

%get angle magnitudes
for i = 1:size(sensorData,1)
    Mag(:,i) = sqrt(sum(sensorData{i}.T_Skm1_Sk(:,4:6).^2,2));
    Var(:,i) = sum(sensorData{i}.T_Var_Skm1_Sk(:,4:6),2);
    
    temp = diff(double(sensorData{i}.time));
    times(:,i) = [temp(1);temp]/1000000;
end

%turn variance into a weight 
w = 1./Var;

%get mean mag
mMag = sum(Mag.*w,2)./sum(w,2);
Var = sqrt(Var);

%get points within maxSD sd of the mean
valid = all(abs(repmat(mMag,1,size(Mag,2)) - Mag) < maxSD*Var,2);
valid = and(valid,all(Var < 1,2));

%get points that move at a speed of at least minMove
valid = and(valid,all((Mag./times) > minMove,2));

%get points with negitive variance
for i = 1:size(sensorData,1)
    valid = and(valid,all(sensorData{i}.T_Var_Skm1_Sk > 0,2));
end

%turn valid into index
valid = find(valid);

%remove invalid points
sensorData = SensorDataSubset(sensorData, valid);
