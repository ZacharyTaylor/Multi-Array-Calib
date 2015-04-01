T = V2T(rand(1,6))

[y,p,r] = dcm2angle(inv(T(1:3,1:3)),'ZYX'); outVec = [T(1:3,4)',r,p,y]

TS = eye(4);
TS(1:3,1:3) = inv(angle2dcm(outVec(6),outVec(5),outVec(4),'ZYX'));
TS(1:3,4) = outVec(1:3);

TS - T