x = 0;
for i = -100000:100:100000
    x = x+1;
    i
    a(x) = getError(t,Mag,Var,10000,i);
end

plot(a)