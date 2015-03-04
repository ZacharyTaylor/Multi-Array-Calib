function [ outVar ] = RotVecVar( vec, vecVar, rotVec, rotVar )
%ROTVAR Rotate a vector by a rotation vector where both elements have
%uncertainty, assumes rotVec ~= 0 (output is approximate)

%check validity
if(any(rotVec > 0.05))
    warning('Approximation only valid for small angles');
end

% %perform rotation
% R = V2R(rotVec);
% outVec = R*vec;

%find variance
outVar = vecVar.*rotVar;
outVar = outVar + (CrossMat(rotVec).^2)*vecVar;
outVar = outVar + abs(CrossMat(rotVar))*vec;

end

