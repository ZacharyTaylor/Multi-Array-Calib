%approximate variance transfer for 1/(R-eye(3))

syms A B C dA dB dC
vec = [dA; dB; dC];

%R-eye(3)
dR = [0,0,0;0,0,-vec(1);0,vec(1),0];
dR = dR + [0,0,vec(2);0,0,0;-vec(2),0,0];
dR = dR + [0,-vec(3),0;vec(3),0,0;0,0,0];

%R = sym('R%d%d',[3,3]);

vec = [A; B; C];

%R-eye(3)
R = [0,0,0;0,-(vec(1).^2)/2,-vec(1);0,vec(1),-(vec(1).^2)/2];
R = R + [-(vec(2).^2)/2,0,vec(2);0,0,0;-vec(2),0,-(vec(2).^2)/2];
R = R + [-(vec(3).^2)/2,-vec(3),0;vec(3),-(vec(3).^2)/2,0;0,0,0];
