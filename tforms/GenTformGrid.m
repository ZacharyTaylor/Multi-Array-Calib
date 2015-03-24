function [ TGrid, vTGrid ] = GenTformGrid(tranVec, rotVec, tranVar, rotVar)
%GENTFORMGRID Summary of this function goes here
%   Detailed explanation goes here

TGrid = cell(size(tranVec,1));
vTGrid = cell(size(tranVec,1));

for i = 1:size(TGrid,1);
    for j = 1:size(vTGrid,1);
        if(i<j)
            [TGrid{i,j},vTGrid{i,j}] = IndVar(0.01,@CombTforms,tranVec(i,:),tranVar(i,:),rotVec(i,:),rotVar(i,:),tranVec(j,:),tranVar(j,:),rotVec(j,:),rotVar(j,:));
        end
    end
end

end

function [outVec] = CombTforms(tranVecA, rotVecA, tranVecB, rotVecB)

outVec = T2V(V2T([tranVecA, rotVecA])\V2T([tranVecB, rotVecB]));

end


