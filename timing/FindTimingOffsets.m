function [ offsets, varOff ] = FindTimingOffsets(Mag,Var,t,samples, navFlag)
%FINDTIMINGOFFSETS Finds the timing offsets between a set of sensors
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   Mag- nx1 cell of absolute angle magnitudes for each sensor
%   Var- nx1 cell of the variance of each angle magnitude
%   t- nx1 cell of timestamps each point occours at
%   samples- number of sample points to use in matching
%   navFlag- flag for if a navigation sensor (affects Var treatment)
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   offsets- nx1 vector of sensor time offsets
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
validateattributes(Mag,{'cell'},{'vector'});
validateattributes(Var,{'cell'},{'vector','numel',length(Mag)});
validateattributes(t,{'cell'},{'vector','numel',length(Mag)});
validateattributes(samples,{'numeric'},{'scalar','nonzero','positive','integer'});
validateattributes(navFlag,{'logical'},{'vector','numel',length(Mag)});

dTest = cell(size(Mag));

for i = 1:length(Mag)
    validateattributes(Mag{i},{'numeric'},{'vector'});
    validateattributes(Var{i},{'numeric'},{'vector','numel',length(Mag{i})});
    validateattributes(t{i},{'numeric'},{'vector','numel',length(Mag{i})});
    
    dTest{i} = zeros(1,samples);
end

%get overlap
% offsets = [1000*ones(length(t)-1,1),ones(length(t)-1,1)];
% for i = 2:length(Mag)
%     rt = t([1,i]);
%     rMag = Mag([1,i]);
%     rVar = Var([1,i]);
%     rNavFlag = navFlag([i,1]);
%     rdTest = dTest([i,1]);
%     offsets(i-1,:) = fminsearch(@(offset) sum(GetTimingError(rt,rMag,rVar,10000,offset, rNavFlag, rdTest)),offsets(i-1,:));
% end

offsets = [1000*ones(length(t)-1,1),ones(length(t)-1,1)];
for i = 2:length(Mag)
    
    %find overlapping regions
    tMin = max(t{1}(1),t{i}(1));
    tMax = min(t{1}(end),t{i}(end));
    
    %sample overlapping area
    tI = tMin:(tMax-tMin)/(samples):tMax;

    %interpolate points
    a = interp1(t{1},Mag{1},tI,'pchip');
    b = interp1(t{i},Mag{i},tI,'pchip');
    %get difference
    a = diff(a) + dTest{1};
    b = diff(b) + dTest{i};
    
    temp = normxcorr2(abs(a), abs(b));
    [~,temp] = max(temp);
    offsets(i-1,1) = (temp - length(a(:)))*median(diff(tI));
end

offsets = fminsearch(@(offset) sum(GetTimingError(t,Mag,Var,samples,offset, navFlag, dTest)),offsets);

varOff = FindTimingVar(offsets, t, Mag, Var, navFlag, dTest, samples);

offsets = [0,1;offsets];
varOff = [0,0;varOff];

end