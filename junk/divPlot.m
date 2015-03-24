samples = 100000000;

out = 1./(0.01 + 0.05*randn(samples,1));

out = out(out < 100);
out = out(out > -100);

m = mean(out);
s = std(out);

out = hist(out,1000);

out = out./samples;


plot([-49.9:0.1:50],out)
