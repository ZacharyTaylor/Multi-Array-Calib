mu = [2,3];
sigma = [1,1.5;1.5,3];

mu2 = 2;
sigma2 = 2;

samples = 100000;

r = mvnrnd(mu,sigma,samples);
r2 = mvnrnd(mu2,sigma2,samples);

valid = r2 > r(:,2);
r2 = r2(valid,1);

hist(r2,100);