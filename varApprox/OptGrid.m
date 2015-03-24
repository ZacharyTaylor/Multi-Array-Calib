function [ finalVec, finalVar ] = OptGrid( TGrid, vTGrid )
%OPTGRID Summary of this function goes here
%   Detailed explanation goes here

finalVec = zeros(size(TGrid,1),6);
finalVar = inf*ones(size(TGrid,1),6);

findVec = @(A,B) T2V(V2T(S2V(A))*V2T(S2V(B)));

for i = 1:size(TGrid,1);
    TGrid{i,i} = zeros(1,7);
    vTGrid{i,i} = zeros(1,7);
end

for i = 1:size(TGrid,1);
    for j = 1:size(vTGrid,1);
        if(i < j)
            [A,VA] = IndVar(0.01,findVec,TGrid{1,i},vTGrid{1,i},TGrid{i,j},vTGrid{i,j});
            [finalVec(j,:),finalVar(j,:)] = CombEst(finalVec(j,:),finalVar(j,:),A,VA);
        end
    end
end

finalVar(1,:) = 0;

finalVar(~isfinite(finalVec)) = inf;
finalVec(~isfinite(finalVec)) = 0;

end

