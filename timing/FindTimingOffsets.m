function [ offsets, varOff ] = FindTimingOffsets(Mag,Var,t,samples)
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

%get overlap
offsets = fminsearch(@(offset) getError(t,Mag,Var,10000,offset,false),[1000*ones(length(t)-1,1),ones(length(t)-1,1)]);

stepA = 1000;
stepB = 0.00001;
stepC = 0.001;
step = 0;

%get variance
dxx = zeros(length(offsets(:)));
for i = 1:length(offsets(:))
    for j = 1:length(offsets(:))
        if(~mod(i,2))
            stepi = stepB;
        else
            stepi = stepA;
        end
        if(~mod(j,2))
            stepj = stepB;
        else
            stepj = stepA;
        end
        
        temp = offsets; 
        temp(j) = temp(j) + stepj;
        temp(i) = temp(i) + stepi;
        f1 = getError(t,Mag,Var,10000,temp,false);

        temp = offsets; 
        temp(j) = temp(j) + stepj;
        temp(i) = temp(i) - stepi;
        f2 = getError(t,Mag,Var,10000,temp,false);

        temp = offsets; 
        temp(j) = temp(j) - stepj;
        temp(i) = temp(i) + stepi;
        f3 = getError(t,Mag,Var,10000,temp,false);

        temp = offsets; 
        temp(j) = temp(j) - stepj;
        temp(i) = temp(i) - stepi;
        f4 = getError(t,Mag,Var,10000,temp,false);

        dxx(i,j) = (f1-f2-f3+f4)/(4*stepi*stepj);
    end
end

dx = zeros(length(offsets(:)),1);
for i = 1:length(offsets(:))
    if(~mod(i,2))
        step = stepB;
    else
        step = stepA;
    end
        
    temp = offsets; 
    temp(i) = temp(i) + step;
    f1 = getError(t,Mag,Var,10000,temp,false);

    temp = offsets; 
    temp(i) = temp(i) - step;
    f2 = getError(t,Mag,Var,10000,temp,false);

    dx(i) = (f1-f2)/(2*step);
end

dz = cell(length(Mag),1);
for i = 1:length(Mag)
    step = stepC;
    temp = Mag;
    temp{i} = temp{i} + step;
    [f1,v1] = getError(t,temp,Var,10000,offsets,true);

    temp = Mag;
    temp{i} = temp{i} - step;
    [f2,v2] = getError(t,temp,Var,10000,offsets,true);

    %find overlapping regions
    tMin = 0;
    tMax = inf;
    for j = 1:length(t)
        tMin = max(tMin,t{j}(1));
        tMax = min(tMax,t{j}(end));
    end

    %sample overlapping area
    tI = tMin:(tMax-tMin)/10000:tMax; tI = tI(2:end);
    
    f1 = interp1(tI',f1',t{i},'pchip');
    f2 = interp1(tI',f2',t{i},'pchip');
     
    %valid = and(v1,v2);
    dz{i} = (f1'-f2')/(2*step);
    %dz(~valid,j,k) = 0;
end
dz = cell2mat(dz')';

dxz = zeros(length(dx(:)),length(dz(:)));
for i = 1:size(dx(:),1)
    for j = 1:size(dz(:),1)
        dxz(i,j) = dx(i) + dz(j);
    end
end

v = cell2mat(Var);

d = dxx\dxz;
d = (d.*repmat(v(:)',size(d,1),1))*d';
varOff = reshape(diag(d),2,[])';

offsets = [0,1;offsets];
varOff = [0,0;varOff];

end

function [err, valid] = getError(t,Mag,Var,samples,offset, vect)

    offset = [0,1;offset];
    
    %find overlapping regions
    tMin = 0;
    tMax = inf;
    for i = 1:length(t)
        t{i} = t{i} - offset(i,1);
        tMin = max(tMin,t{i}(1));
        tMax = min(tMax,t{i}(end));
    end
    
    for i = 1:length(t)
        t{i} = t{i} - tMin;
        t{i} = t{i}*offset(i,2);
    end
    
    tMax = tMax - tMin;
    tMin = 0;

    %sample overlapping area
    tI = tMin:(tMax-tMin)/(samples):tMax;

    for i = 1:length(Mag)
        %interpolate points
        Mag{i} = interp1(t{i},Mag{i},tI,'pchip');
        %get difference
        Mag{i} = diff(Mag{i});

        %get weighting from variance
        Var{i} = interp1(t{i},Var{i},tI,'pchip');
        Var{i} = 1./abs(diff(Var{i}));
    end
    
    wM = zeros(1,samples);
    w = zeros(1,samples);
    %get weighted mean
    for i = 1:length(Mag)
        wM = wM + Mag{i}.*Var{i};
        w = w + Var{i};
    end
    
    wM = wM./w;
    
    err = zeros(1,samples);
    for i = 2:length(Mag)
        err = err + log(((wM - Mag{i}).^2).*(w + Var{i}));
    end
    
    err = err(isfinite(err));
    
    valid = false(size(err));
    [~,idx] = sort(err,'ascend');
    idx = idx(floor(size(idx(:),1)*0.25):floor(size(idx(:),1)*0.75));
    valid(idx) = true;
    if(~vect)
        err = sum(err(valid));
    end
end