a = zeros(1000,1);
b = zeros(1000,1);

for i = 1:1000
    out = V2R([0,0,i/10000]);
    b(i) = 1-out(1);
    a(i) = out(2);
end

plot([a,sqrt(2*b)])