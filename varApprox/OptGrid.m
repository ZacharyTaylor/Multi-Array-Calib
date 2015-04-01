function [ finalVec, finalVar ] = OptGrid( TGrid, vTGrid )
%OPTGRID converts an inconsistent grid of transformations to a vector of
%   transforamtions wrt sensor 1.
%--------------------------------------------------------------------------
%   Inputs:
%--------------------------------------------------------------------------
%   TGrid- nxn cell of sensor transformations. TGrid(i,j) is the
%   transformation from sensor i to sensor j. Transforms are stored in
%   scaled vector form. Any unknown transformation is left empty
%   vTGrid- nxn cell of the transforamiotn variances
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   finalVec- nx6 array of sensor transoforamation vectors wrt sensor 1
%   finalVar- nx6 array of corrosponding variances
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
validateattributes(TGrid,{'cell'},{'square'});
validateattributes(vTGrid,{'cell'},{'size',size(TGrid)});

for i = 1:length(TGrid(:))
    if(~isempty(TGrid{i}))
        validateattributes(TGrid{i},{'double'},{'size',[1,7]});
        validateattributes(vTGrid{i},{'double'},{'size',[1,7]});
    end
end

%setup output
finalVec = zeros(size(TGrid,1),6);
finalVar = inf*ones(size(TGrid,1),6);

%conversion function
findVec = @(A,B) T2V(V2T(S2V(A))*V2T(S2V(B)));

for i = 1:size(TGrid,1);
    TGrid{i,i} = zeros(1,7);
    vTGrid{i,i} = zeros(1,7);
end

for i = 1:size(TGrid,1);
    for j = 1:size(vTGrid,1);
        if(i < j)
            %find sol and variance
            [A,VA] = IndVar(0.01,findVec,TGrid{1,i},vTGrid{1,i},TGrid{i,j},vTGrid{i,j});
            %combine
            [finalVec(j,:),finalVar(j,:)] = CombEst(finalVec(j,:),finalVar(j,:),A,VA);
        end
    end
end

%handle cases of 0 and inf variance
finalVar(1,:) = 0;
finalVar(~isfinite(finalVec)) = inf;
finalVec(~isfinite(finalVec)) = 0;

end

