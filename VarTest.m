r = [1,0.5,0.3];
t = [4,5,25];

rv = [0.01,0.02,0.005];
tv = [0.3,0.2,0.1];
%tv = [0,0,0];

out = zeros(10000,3);

for i = 1:10000
    R = V2R(r + sqrt(rv).*randn(1,3));
    T = t + sqrt(tv).*randn(1,3);

    out(i,:) = R*T';
end

var(out)

out = zeros(3,3);
offset = 0.001;
base = (V2R(r)*t')';
for i = 1:6
    ir = r;
    it = t;
    v = 0;
    if(i > 3)
        ir(i-3) = ir(i-3) + offset;
        v = rv(i-3);
    else
        it(i) = it(i) + offset;
        v = tv(i);
    end

    out(i,:) = V2R(ir)*it';
    out(i,:) = out(i,:) - base;
    out(i,:) = v.*(out(i,:)./offset).^2;
end

output = @(r,t) (V2R(r)*t');
[ val, var ] = IndVar(0.001, output, r,rv,t,tv )



%(abs(CrossMat(rv))*((V2R(r)*t').^2))' + diag(V2R(r)*diag(tv)*V2R(r)')'