%approximate variance transfer for 1/(R-eye(3))

vecR = [0.5 0.2 0.7];
varR = [0.1 0.05, 0.003];

vecT = [0.8 0.5 0.3];
varT = [0.2 0.2, 0.03];

samples = 1000;
out = zeros(3,samples);
for i = 1:samples
    out(:,i) = V2R(vecR + randn(1,3).*sqrt(varR))*(vecT + randn(1,3).*sqrt(varT))';
end

mS = mean(out,2);
vS = var(out,[],2);

diff = 0.0001;
mE = V2R(vecR(1:3))*vecT';
out2 = zeros(3,6);
temp = [vecR,vecT];
tempV = [varR,varT];
for i = 1:6
    
    tV = zeros(size(tempV));
    tV(i) = diff;

    out2(:,i) = tempV(i).*((V2R(vecR(1:3) + tV(1:3))*(vecT + tV(4:6))' - mE).^2);
end

out2 = sum(out2,2)./(diff.^2);

    
tV = sqrt(tempV)*diff;
out3 = (V2R(vecR(1:3) + tV(1:3))*(vecT + tV(4:6))' - mE).^2;
out3 = out3./(diff.^2);

