function [ outVar ] = InvRotVecVar( rotVec, rotVar )
%ROTVAR finds the variance when you invert a rotation vector

%check validity
if(any(rotVec > 0.05))
    warning('Approximation only valid for small angles');
end

%find variance
outVar = rotVar./(rotVec.^4);

end

