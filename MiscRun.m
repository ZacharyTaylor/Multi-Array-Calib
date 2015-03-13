%Used when I want to profile a random section of the script
tic
temp = eq1\eq2;
        toc
        tic
        fminsearch(@(T) sum((eq1*temp-eq2).^2),[0;0;0;1]);
toc