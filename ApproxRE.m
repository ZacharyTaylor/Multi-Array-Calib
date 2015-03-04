%approximate variance transfer for 1/(R-eye(3))

vec = [0.05, 0.01, 0.03];

%R-eye(3)
R = [0,0,0;0,-(vec(1).^2)/2,-vec(1);0,vec(1),-(vec(1).^2)/2];
R = R + [-(vec(2).^2)/2,0,vec(2);0,0,0;-vec(2),0,-(vec(2).^2)/2];
R = R + [-(vec(3).^2)/2,-vec(3),0;vec(3),-(vec(3).^2)/2,0;0,0,0];