function [ offsets ] = FindTimingOffsets(Mag,Var,t,samples)
%FINDTIMINGOFFSETS Finds the timing offsets between a set of sensors
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   Mag- nx1 cell of absolute angle magnitudes for each sensor
%   Var- nx1 cell of the variance of each angle magnitude
%   t- nx1 cell of timestamps each point occours at
%   samples- number of sample points to use in matching
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

for i = 1:length(Mag)
    validateattributes(Mag{i},{'numeric'},{'vector'});
    validateattributes(Var{i},{'numeric'},{'vector','numel',length(Mag{i})});
    validateattributes(t{i},{'numeric'},{'vector','numel',length(Mag{i})});
end

%find overlapping regions
tMin = 0;
tMax = inf;
for i = 1:length(t)
    tMin = max(tMin,t{i}(1));
    tMax = min(tMax,t{i}(end));
end

%sample overlapping area
tI = tMin:(tMax-tMin)/(samples):tMax;

for i = 1:length(Mag)
    
    %interpolate points
    Mag{i} = interp1(t{i},Mag{i},tI,'pchip');
    %get difference
    Mag{i} = abs(diff(Mag{i}));
    
    %get weighting from variance
    Var{i} = interp1(t{i},Var{i},tI,'pchip');
    Var{i} = 1./abs(diff(Var{i}));
    
%     %take largest 10% Weights to be outliers and ignore
%     [~,idx] = sort(Var{i},'descend');
%     idx = idx(1:ceil(length(idx)/5));
%     Var{i}(idx) = 0;
%     
%     %take largest 10% to be outliers and ignore
%     [~,idx] = sort(Mag{i},'descend');
%     idx = idx(1:ceil(length(idx)/5));
%     Var{i}(idx) = 0;

    %histogram equalize
    %Mag{i} = MyHistEq(Mag{i});
    
    %remove bias
    Mag{i} = (Mag{i} - mean(Mag{i})) / std(Mag{i});
    
    Mag{i} = medfilt1(Mag{i},100);
    Var{i} = medfilt1(Var{i},100);
end

%get all possible comibnations of sensors
P = nchoosek(1:length(Mag),2);
v = zeros(size(P,1)+1,1);
x = zeros(size(P,1)+1,length(Mag));

%for every sensor pair
for i = 1:size(P,1)
    %find offset between sensors
    temp = wncc(Mag{P(i,1)},Mag{P(i,2)},min(Var{P(i,1)},Var{P(i,2)}));
    [val,idx] = imax(temp);
    v(i) = val*(ceil(samples/2) - idx);
    x(i,P(i,1)) =val;
    x(i,P(i,2)) = -val;
end

x(end,1) = 1;

%solve matrix for offsets
offsets = ((tMax-tMin)/samples)*(x\v);

%set first sensor to zero offset
offsets = offsets - offsets(1);

end

