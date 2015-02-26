syms V1 V2 V3
assume(V1,'real');
assume(V2,'real');
assume(V3,'real');
V = [V1 V2 V3];

n = sqrt(sum(V.^2));

% build the rotation matrix
s = sin(n);
c = cos(n);
t = 1 - c;

x = V(1);
y = V(2);
z = V(3);
R = [ ...
     t*x*x + c,    t*x*y - s*z,  t*x*z + s*y; ...
     t*x*y + s*z,  t*y*y + c,    t*y*z - s*x; ...
     t*x*z - s*y,  t*y*z + s*x,  t*z*z + c ...
    ];
