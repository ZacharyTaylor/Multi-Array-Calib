function [out] = GetTimingError(t,Mag,Var,samples,offset, navFlag, dTest)

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
        Mag{i} = abs(diff(Mag{i}) + dTest{i});

        %get weighting from variance
        Var{i} = interp1(t{i},Var{i},tI,'pchip');
        if(navFlag(i))
            Var{i} = abs(Var{i}(:,2:end));
        else
            Var{i} = abs(diff(Var{i}));
        end
        Var{i} = 1./Var{i};
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
    
    err(~isfinite(err)) = 0;
    
    out = zeros(size(err));
    [~,idx] = sort(err,'ascend');
    %idx = idx(floor(size(idx(:),1)*0.25):floor(size(idx(:),1)*0.75));
    idx = idx(1:floor(size(idx(:),1)*0.75));
    out(idx) = err(idx);

end