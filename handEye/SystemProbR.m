function [ prob, valid ] = SystemProbR( RData, vRData, estVec, retVecFlag )
%SYSTEMPROBR uses variance to find a measure of the systems probablity of
%   being correct (lower score == better)
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   sensorData- nx1 cell containing sensor data sturcts
%   estVec- (n-1)x3 matrix of rotations for each sensor (1st sensor info
%       not given as it will just be 0,0,0)
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   prob- measure of system likelihood (note not an actual probabilty)
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
validateattributes(RData,{'double'},{'3d','ncols',3});
validateattributes(vRData,{'double'},{'size',size(RData)});
validateattributes(estVec,{'numeric'},{'size',[size(RData,3)-1,3]});

%set first element to zeros
estVec = [0,0,0;estVec];

%find probablity of system
i = 0;
prob = 0;
for a = 1:size(RData,3)
    for b = 1:size(RData,3)
        %ensure no repeats
        if(a < b)
            i = i+1;
            %get rotation and variance
            VA = vRData(:,:,a)';
            VB = vRData(:,:,b)';
            
            RA = RData(:,:,a)';
            RB = RData(:,:,b)';
            
            %find position error
            temp = logpdfR(RA,RB,VA,VB,estVec(a,:),estVec(b,:));
            
            %add error
            prob= prob + temp;
        end
    end
end

[~,idx] = sort(prob,'descend');
valid = true(size(prob));
valid(idx(floor(size(prob,1)*0.75):end)) = false;

if(~retVecFlag)
    prob = -sum(prob(valid));
end

end

