function [ TMat ] = Grid2Mat(TGrid)
%GENTFORMGRID generates transformation matricies from the output tgrid

for i = 1:size(TGrid,1)
    TGrid{i,i} = zeros(1,6);
end

TMat = TGrid;
for i = 1:size(TMat,1)
    for j = 1:size(TMat,2)
        if(isempty(TMat{i,j}))
            TMat{i,j} = inv(V2T(TGrid{j,i}));
        else
            TMat{i,j} = V2T(TGrid{i,j});
        end
    end
end

