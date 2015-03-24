function [ varVec ] = RotVarMat( varVec, R )
%ROTVAR Rotate variance vector by an exact rotation matrix R

varVec = R*varVec*(R');

end

